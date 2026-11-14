/* d3d9_sm.c — shader bytecode (vs / ps 1.x - 3.0) to GLSL ES text for host/win/shc: every instruction becomes one assignment through its write mask.
 * Names: inputs a_u<usage>_<index>, outputs v_c<n>, v_t<n>, v_fog, v_u<usage>_<index> (d3d9_ff.c links by them); constants vc<n> / pc<n>, or dc<n> for one the shader defines itself.
 * c[a0.x + n] with a0 loaded from a constant is one register per draw: a uniform vr<i> that d3_prog_uniforms fills. With a0 loaded from anything else (skinning) a shader reads every constant from the
 * array vca, and the compiler makes a run-time index of it (LDPX); so does c[aL + n].
 * A subroutine has no hardware behind it and a shader's call stack is at most four deep: walk() writes its code at each call. p0 is the vec4 g2_p of 1s and 0s.
 * A loop count from SetVertexShaderConstantI / SetPixelShaderConstantI is a macro G2I<n>X/Y/Z, which d3d9_ff.c defines per link: the compiler unrolls loops, so each count is a program of its own.
 * ps 1.x's bump ops read the stage's BUMPENVMAT as a uniform pm<n> and its BUMPENVLSCALE / LOFFSET as pl<n>; texldd works the level out from the texture's size, a uniform pz<n>.
 * Every sample goes through a macro G2S<n>, which d3d9_ff.c defines per link: bump formats are stored biased (the TMU has no signed format).
 * Not translated, and said so at creation: a ret inside a branch or a loop, which Direct3D's validator does not let through either. */
#include "d3d9_internal.h"

enum { R_TEMP, R_INPUT, R_CONST, R_ADDR, R_RASTOUT, R_ATTROUT, R_OUTPUT, R_CONSTINT, R_COLOROUT, R_DEPTHOUT, R_SAMPLER, R_CONST2, R_CONST3, R_CONST4, R_CONSTBOOL, R_LOOP, R_TEMP16, R_MISC, R_LABEL, R_PREDICATE };
enum { O_NOP, O_MOV, O_ADD, O_SUB, O_MAD, O_MUL, O_RCP, O_RSQ, O_DP3, O_DP4, O_MIN, O_MAX, O_SLT, O_SGE, O_EXP, O_LOG, O_LIT, O_DST, O_LRP, O_FRC, O_M4x4, O_M4x3, O_M3x4, O_M3x3, O_M3x2, O_CALL, O_CALLNZ, O_LOOP, O_RET,
       O_ENDLOOP, O_LABEL, O_DCL, O_POW, O_CRS, O_SGN, O_ABS, O_NRM, O_SINCOS, O_REP, O_ENDREP, O_IF, O_IFC, O_ELSE, O_ENDIF, O_BREAK, O_BREAKC, O_MOVA, O_DEFB, O_DEFI,
       O_TEXCOORD = 64, O_TEXKILL, O_TEX, O_TEXBEM, O_TEXBEML, O_TEXREG2AR, O_TEXREG2GB, O_TEXM3x2PAD, O_TEXM3x2TEX, O_TEXM3x3PAD, O_TEXM3x3TEX, O_TEXM3x3SPEC = 76, O_TEXM3x3VSPEC, O_EXPP, O_LOGP, O_CND, O_DEF,
       O_TEXREG2RGB, O_TEXDP3TEX, O_TEXM3x2DEPTH, O_TEXDP3, O_TEXM3x3, O_TEXDEPTH, O_CMP, O_BEM, O_DP2ADD, O_DSX, O_DSY, O_TEXLDD, O_SETP, O_TEXLDL, O_BREAKP, O_PHASE = 0xFFFD, O_COMMENT = 0xFFFE, O_END = 0xFFFF };
#define INS_PREDICATED 0x10000000u

typedef struct sm {
    d3shader *s;
    const DWORD *t, *end;
    int ps, major, minor;
    d3sb decl, body;
    uint8_t used_c[PS_CONSTS > VS_CONSTS ? PS_CONSTS : VS_CONSTS], def_c[VS_CONSTS], used_i[16], used_b[16], def_i[16], def_b[16];
    int defi[16][4];
    uint8_t temps[32], inputs[32], tex_tmp[8], samp[16], out_c[2], out_t[10], out_fog, out_o[12], in_t[10], in_c[2];
    uint8_t out_o_usage[12], out_o_index[12], in_usage[16], in_index[16], in_dcl[16];
    uint8_t out_c_mask[2], out_t_mask[10], out_o_mask[12];
    uint8_t bem[8], lum[8], tsize[8];           /* the uniforms pm<n>, pl<n>, pz<n> */
    int depth_out, loops, reps, color_out, out_extra, pred;
    int a0_reg[4], a0_comp[4], a0_round[4];     /* where each component of a0 came from: the constant register, A0_VERTEX a per-vertex value, -1 not loaded yet */
    int a0_vertex;                              /* some load of a0 is per-vertex: the constants are read from vca */
    uint8_t a0_var[4];                          /* the int variables a0x .. a0w are in use */
    const DWORD *label[32];                     /* the token after each label */
    int ins_n, zr_at;                           /* instructions walked so far; the one G2_ZR goes before, -1: none (pos_scan) */
    DWORD zr_x;                                 /* its vector operand */
    char err[160];
} sm;
#define A0_VERTEX (-2)

static void fail(sm *m, const char *what) { if (!m->err[0]) snprintf(m->err, sizeof m->err, "%s", what); }
static int reg_type(DWORD p) { return (int)((p >> 28 & 7) | (p >> 8 & 0x18)); }
static int reg_num(DWORD p) { return (int)(p & 0x7FF); }

static void in_name(sm *m, int i, char *o, size_t n)
{
    if (!m->ps) { snprintf(o, n, "v%d", i); return; }
    if (m->major >= 3) {
        int u = m->in_usage[i], x = m->in_index[i];
        if (u == D3DDECLUSAGE_COLOR) snprintf(o, n, "v_c%d", x);
        else if (u == D3DDECLUSAGE_TEXCOORD) snprintf(o, n, "v_t%d", x);
        else snprintf(o, n, "v_u%d_%d", u, x);
        return;
    }
    snprintf(o, n, "v_c%d", i);
    if (i < 2) m->in_c[i] = 1;
}

/* a register as a vec4 expression */
static void reg_text(sm *m, DWORD p, char *o, size_t n, int is_dst)
{
    int ty = reg_type(p), r = reg_num(p);
    switch (ty) {
    case R_TEMP: if (r < 32) m->temps[r] = 1; snprintf(o, n, "r%d", r); break;
    case R_INPUT: if (r < 32) m->inputs[r] = 1; in_name(m, r, o, n); break;
    case R_CONST:
        if (r >= (m->ps ? PS_CONSTS : VS_CONSTS)) { fail(m, "constant register out of range"); snprintf(o, n, "vec4(0.0)"); break; }
        if (!m->ps && m->def_c[r]) snprintf(o, n, "dc%d", r);
        else if (m->ps && r < VS_CONSTS && m->def_c[r]) snprintf(o, n, "dc%d", r);
        else if (m->a0_vertex) {
            if (r >= VCA_N) fail(m, "a constant beyond the array a per-vertex a0 indexes");
            snprintf(o, n, "vca[%d]", r);
        }
        else { m->used_c[r] = 1; snprintf(o, n, "%cc%d", m->ps ? 'p' : 'v', r); }
        break;
    case R_ADDR:                                                /* a0 in a vertex shader, t<n> in a pixel shader */
        if (!m->ps) { fail(m, "the address register: the vertex ISA has no indirect load"); snprintf(o, n, "vec4(0.0)"); break; }
        if (m->major == 1 && m->minor < 4) { m->tex_tmp[r & 7] = 1; snprintf(o, n, "t%d", r); }
        else { m->in_t[r & 7] = 1; snprintf(o, n, "v_t%d", r); }
        break;
    case R_RASTOUT:
        if (r == 0) snprintf(o, n, "g2p");
        else if (r == 1) { m->out_fog = 1; snprintf(o, n, "v_fogv"); }
        else snprintf(o, n, "v_psz");
        break;
    case R_ATTROUT: if (r < 2) m->out_c[r] = 1; snprintf(o, n, "v_c%d", r); break;
    case R_OUTPUT:
        if (m->major >= 3) {
            int u = r < 12 ? m->out_o_usage[r] : 0, x = r < 12 ? m->out_o_index[r] : 0;
            if (r < 12) m->out_o[r] = 1;
            if (u == D3DDECLUSAGE_POSITION) snprintf(o, n, "g2p");
            else if (u == D3DDECLUSAGE_COLOR) snprintf(o, n, "v_c%d", x);
            else if (u == D3DDECLUSAGE_TEXCOORD) snprintf(o, n, "v_t%d", x);
            else if (u == D3DDECLUSAGE_FOG) { m->out_fog = 1; snprintf(o, n, "v_fogv"); }
            else if (u == D3DDECLUSAGE_PSIZE) snprintf(o, n, "v_psz");
            else snprintf(o, n, "v_u%d_%d", u, x);
        } else {
            if (r < 10) m->out_t[r] = 1;
            snprintf(o, n, "v_t%d", r);
        }
        break;
    case R_CONSTINT:
        if (m->def_i[r & 15]) snprintf(o, n, "vec4(%d.0, %d.0, %d.0, %d.0)", m->defi[r & 15][0], m->defi[r & 15][1], m->defi[r & 15][2], m->defi[r & 15][3]);
        else { m->used_i[r & 15] = 1; snprintf(o, n, "%ci%d", m->ps ? 'p' : 'v', r); }
        break;
    case R_COLOROUT:                                            /* NumSimultaneousRTs is 1: Direct3D drops what a shader writes to an output with no target */
        if (r) { m->out_extra = 1; snprintf(o, n, "g2_drop"); }
        else { m->color_out = 1; snprintf(o, n, "g2_out"); }
        break;
    case R_DEPTHOUT: m->depth_out = 1; snprintf(o, n, "g2_depth"); break;
    case R_MISC:
        if (r == 0) snprintf(o, n, "vec4(gl_FragCoord.xy - vec2(0.5), 0.0, 1.0)");
        else snprintf(o, n, "vec4(gl_FrontFacing ? 1.0 : -1.0)");
        break;
    case R_LOOP: snprintf(o, n, "vec4(float(aL))"); break;
    case R_CONSTBOOL:
        if (m->def_b[r & 15]) snprintf(o, n, "vec4(%d.0)", m->def_b[r & 15] - 1);
        else { m->used_b[r & 15] = 1; snprintf(o, n, "%cb%d", m->ps ? 'p' : 'v', r); }
        break;
    case R_PREDICATE: m->pred = 1; snprintf(o, n, "g2_p"); break;
    default: fail(m, "a register type this translator does not take"); snprintf(o, n, "vec4(0.0)"); break;
    }
    (void)is_dst;
}

static void reg_at(char *o, size_t n, int base, const char *pre, int add) { snprintf(o, n, "%s%d", pre, base + add); }

/* mov / mova into a0: remembered, the reads below decide whether it can be followed */
static void a0_load(sm *m, DWORD d, DWORD src, int rel, int round)
{
    int k;
    for (k = 0; k < 4; k++) {
        if (!(d >> (16 + k) & 1)) continue;
        m->a0_reg[k] = reg_type(src) == R_CONST && !rel && !(src >> 24 & 15) ? reg_num(src) : A0_VERTEX;
        m->a0_comp[k] = (int)(src >> (16 + 2 * k) & 3);
        m->a0_round[k] = round;
    }
}

/* the uniform that stands for c[a0.<comp> + base] */
static int rel_slot(sm *m, int comp, int type, int base)
{
    d3shader *s = m->s;
    int reg = m->a0_reg[comp], i;
    if (type != R_CONST || reg < 0) { fail(m, "relative addressing of something other than the constants, or by an a0 never loaded"); return -1; }
    for (i = 0; i < s->nrel; i++) if (s->rel[i].reg == reg && s->rel[i].comp == m->a0_comp[comp] && s->rel[i].base == base && s->rel[i].round == m->a0_round[comp]) return i;
    if (s->nrel >= MAX_REL) { fail(m, "more than 16 relative constants"); return -1; }
    s->rel[s->nrel].reg = (short)reg; s->rel[s->nrel].comp = (short)m->a0_comp[comp]; s->rel[s->nrel].base = (short)base; s->rel[s->nrel].round = (short)m->a0_round[comp];
    return s->nrel++;
}

#define REL_LOOP 5                              /* srel: addressed by aL, not a0 */

/* a source operand, modifiers and swizzle applied; add: the register number offset of m4x4 and friends; rel: 0, 1 + the component of a0 it is addressed by, or REL_LOOP */
static void src_text(sm *m, DWORD p, int rel, int add, char *o, size_t n)
{
    char r[64], sw[8];
    DWORD s = p >> 16 & 255;
    int mod = (int)(p >> 24 & 15), i;
    DWORD q = p;
    if (add) q = (p & ~(DWORD)0x7FF) | (DWORD)(reg_num(p) + add);
    if (rel == REL_LOOP) {
        if (reg_type(p) != R_CONST || m->ps) { fail(m, "aL addressing something other than a vertex shader's constants"); snprintf(o, n, "vec4(0.0)"); return; }
        snprintf(r, sizeof r, "vca[aL + %d]", reg_num(q));
    }
    else if (rel && m->a0_reg[rel - 1] == A0_VERTEX && reg_type(p) == R_CONST) snprintf(r, sizeof r, "vca[a0%c + %d]", "xyzw"[rel - 1], reg_num(q));
    else if (rel) {
        int slot = rel_slot(m, rel - 1, reg_type(p), reg_num(q));
        if (slot < 0) { snprintf(o, n, "vec4(0.0)"); return; }
        snprintf(r, sizeof r, "vr%d", slot);
    } else reg_text(m, q, r, sizeof r, 0);
    (void)reg_at;
    if (s == 0xE4) sw[0] = 0;
    else {
        sw[0] = '.';
        for (i = 0; i < 4; i++) sw[1 + i] = "xyzw"[s >> (2 * i) & 3];
        sw[5] = 0;
    }
    switch (mod) {
    case 0: snprintf(o, n, "%s%s", r, sw); break;
    case 1: snprintf(o, n, "(-%s%s)", r, sw); break;
    case 2: snprintf(o, n, "(%s%s - vec4(0.5))", r, sw); break;
    case 3: snprintf(o, n, "(vec4(0.5) - %s%s)", r, sw); break;
    case 4: snprintf(o, n, "(2.0 * (%s%s - vec4(0.5)))", r, sw); break;
    case 5: snprintf(o, n, "(-2.0 * (%s%s - vec4(0.5)))", r, sw); break;
    case 6: snprintf(o, n, "(vec4(1.0) - %s%s)", r, sw); break;
    case 7: snprintf(o, n, "(2.0 * %s%s)", r, sw); break;
    case 8: snprintf(o, n, "(-2.0 * %s%s)", r, sw); break;
    case 9: snprintf(o, n, "(%s%s / %s%s.z)", r, sw, r, sw); break;              /* _dz, _dw of ps_1_4's texld and texcrd */
    case 10: snprintf(o, n, "(%s%s / %s%s.w)", r, sw, r, sw); break;
    case 11: snprintf(o, n, "abs(%s%s)", r, sw); break;
    case 12: snprintf(o, n, "(-abs(%s%s))", r, sw); break;
    case 13: snprintf(o, n, "(vec4(1.0) - %s%s)", r, sw); break;                 /* !b0, !p0 */
    default: fail(m, "a source modifier this translator does not take"); snprintf(o, n, "vec4(0.0)"); break;
    }
}

static int op_is_scalar_src(int op) { return op == O_RCP || op == O_RSQ || op == O_EXP || op == O_LOG || op == O_POW || op == O_EXPP || op == O_LOGP || op == O_SINCOS; }

typedef struct ins { DWORD tok; int op; DWORD d, s[4], pred; int srel[4], ns, has_d; } ins;

static int instr_shape(int op, int major, int minor, int *has_d, int *ns)
{
    *has_d = 1;
    switch (op) {
    case O_NOP: case O_ELSE: case O_ENDIF: case O_ENDLOOP: case O_ENDREP: case O_RET: case O_BREAK: *has_d = 0; *ns = 0; return 1;
    case O_MOV: case O_RCP: case O_RSQ: case O_EXP: case O_LOG: case O_LIT: case O_FRC: case O_ABS: case O_NRM: case O_MOVA: case O_EXPP: case O_LOGP: case O_DSX: case O_DSY: *ns = 1; return 1;
    case O_ADD: case O_SUB: case O_MUL: case O_DP3: case O_DP4: case O_MIN: case O_MAX: case O_SLT: case O_SGE: case O_DST: case O_M4x4: case O_M4x3: case O_M3x4: case O_M3x3: case O_M3x2: case O_POW: case O_CRS: case O_BEM: case O_SETP: *ns = 2; return 1;
    case O_MAD: case O_LRP: case O_SGN: case O_CMP: case O_CND: case O_DP2ADD: *ns = 3; return 1;
    case O_SINCOS: *ns = major >= 3 ? 1 : 3; return 1;
    case O_TEX: *ns = major >= 2 ? 2 : (major == 1 && minor == 4) ? 1 : 0; return 1;
    case O_TEXCOORD: *ns = (major == 1 && minor == 4) ? 1 : 0; return 1;
    case O_TEXKILL: case O_TEXDEPTH: *ns = 0; return 1;
    case O_TEXBEM: case O_TEXBEML: case O_TEXREG2AR: case O_TEXREG2GB: case O_TEXREG2RGB: case O_TEXM3x2PAD: case O_TEXM3x2TEX: case O_TEXM3x2DEPTH: case O_TEXM3x3PAD: case O_TEXM3x3TEX: case O_TEXM3x3VSPEC: case O_TEXM3x3: case O_TEXDP3: case O_TEXDP3TEX: *ns = 1; return 1;
    case O_TEXM3x3SPEC: *ns = 2; return 1;
    case O_TEXLDL: *ns = 2; return 1;
    case O_TEXLDD: *ns = 4; return 1;
    case O_IF: case O_REP: case O_BREAKP: case O_CALL: case O_LABEL: *has_d = 0; *ns = 1; return 1;
    case O_IFC: case O_BREAKC: case O_LOOP: case O_CALLNZ: *has_d = 0; *ns = 2; return 1;
    default: return 0;
    }
}

static int read_ins(sm *m, const DWORD **pp, ins *i)
{
    const DWORD *p = *pp;
    int k, len = -1;
    i->tok = *p++;
    i->op = (int)(i->tok & 0xFFFF);
    if (m->major >= 2) len = (int)(i->tok >> 24 & 15);
    if (!instr_shape(i->op, m->major, m->minor, &i->has_d, &i->ns)) { char b[64]; snprintf(b, sizeof b, "opcode %d is not translated", i->op); fail(m, b); return -1; }
    (void)len;
    if (i->has_d) {
        i->d = *p++;
        if (i->d >> 13 & 1) { if (m->major >= 2) p++; fail(m, "a destination with relative addressing"); }
    }
    if (i->tok & INS_PREDICATED) i->pred = *p++;                /* after the destination, before the sources */
    for (k = 0; k < i->ns; k++) {
        i->s[k] = *p++;
        i->srel[k] = (int)(i->s[k] >> 13 & 1);
        if (i->srel[k] && m->major >= 2) {                      /* the token after it names a0's component, or aL; vs 1.x has only a0.x */
            DWORD at = *p++;
            i->srel[k] = reg_type(at) == R_LOOP ? REL_LOOP : 1 + (int)(at >> 16 & 3);
        }
    }
    *pp = p;
    return 0;
}

static void assign(sm *m, const ins *i, const char *expr)
{
    char d[48], mk[8];
    DWORD wm = i->d >> 16 & 15;
    int mod = (int)(i->d >> 20 & 15), sh = (int)(i->d >> 24 & 15), k = 0, ty = reg_type(i->d), r = reg_num(i->d);
    d3sb *b = &m->body;
    double scale = 1.0;
    if (sh) scale = sh < 8 ? (double)(1 << sh) : 1.0 / (double)(1 << (16 - sh));
    reg_text(m, i->d, d, sizeof d, 1);
    if (wm == 15) mk[0] = 0;
    else {
        mk[k++] = '.';
        if (wm & 1) mk[k++] = 'x';
        if (wm & 2) mk[k++] = 'y';
        if (wm & 4) mk[k++] = 'z';
        if (wm & 8) mk[k++] = 'w';
        mk[k] = 0;
    }
    if (ty == R_ATTROUT && r < 2) m->out_c_mask[r] |= (uint8_t)wm;
    if (ty == R_OUTPUT && m->major < 3 && r < 10) m->out_t_mask[r] |= (uint8_t)wm;
    if (ty == R_OUTPUT && m->major >= 3 && r < 12) m->out_o_mask[r] |= (uint8_t)wm;
    if (i->tok & INS_PREDICATED) {                              /* each written component on its own component of p0; the result is worked out once, its inputs may be what it writes */
        char p[80];
        src_text(m, i->pred, 0, 0, p, sizeof p);
        d3sb_printf(b, "    {\n    vec4 g2_t = %s%.6f * (%s)%s;\n", mod & 1 ? "clamp(" : "", scale, expr, mod & 1 ? ", 0.0, 1.0)" : "");
        for (k = 0; k < 4; k++) if (wm >> k & 1) d3sb_printf(b, "    if ((%s).%c != 0.0) %s.%c = g2_t.%c;\n", p, "xyzw"[k], d, "xyzw"[k], "xyzw"[k]);
        d3sb_printf(b, "    }\n");
        return;
    }
    d3sb_printf(b, "    %s%s = ", d, mk);
    if (mod & 1) d3sb_printf(b, "clamp(");
    if (scale != 1.0) d3sb_printf(b, "%.6f * ", scale);
    d3sb_printf(b, "(%s)%s", expr, mk);
    if (mod & 1) d3sb_printf(b, ", 0.0, 1.0)");
    d3sb_printf(b, ";\n");
}

/* the texture a pixel shader samples: how: 0 plain, 1 projected, 2 biased by .w, 3 at level .w */
static void tex_expr(sm *m, int unit, const char *coord, int how, char *o, size_t n)
{
    char t[1700];
    if (unit > 7) { fail(m, "more than 8 samplers"); unit = 0; }
    if (!m->samp[unit]) m->samp[unit] = 1;                      /* ps 1.x declares no samplers: what the stage holds decides, and d3d9_ff.c defines G2T<n> per link */
    if (m->samp[unit] == 1) snprintf(t, sizeof t, "G2T%d(%s)", unit, coord);
    else if (m->samp[unit] == 3) {
        if (how == 3) snprintf(t, sizeof t, "textureCubeLod(s%d, (%s).xyz, (%s).w G2A%d)", unit, coord, coord, unit);
        else if (how == 2) snprintf(t, sizeof t, "textureCube(s%d, (%s).xyz, (%s).w G2A%d)", unit, coord, coord, unit);
        else snprintf(t, sizeof t, "textureCube(s%d, (%s).xyz G2B%d)", unit, coord, unit);
    }
    else if (m->samp[unit] == 4) {
        if (how == 3) snprintf(t, sizeof t, "G2E%d(texture3DLod(s%d, G2C%d((%s).xyz), (%s).w G2A%d), (%s).xyz)", unit, unit, unit, coord, coord, unit, coord);
        else if (how == 1) snprintf(t, sizeof t, "G2E%d(texture3D(s%d, G2C%d((%s).xyz / (%s).w) G2B%d), (%s).xyz / (%s).w)", unit, unit, unit, coord, coord, unit, coord, coord);
        else if (how == 2) snprintf(t, sizeof t, "G2E%d(texture3D(s%d, G2C%d((%s).xyz), (%s).w G2A%d), (%s).xyz)", unit, unit, unit, coord, coord, unit, coord);
        else snprintf(t, sizeof t, "G2E%d(texture3D(s%d, G2C%d((%s).xyz) G2B%d), (%s).xyz)", unit, unit, unit, coord, unit, coord);
    }
    else if (how == 3) snprintf(t, sizeof t, "G2X%d(G2E%d(texture2DLod(s%d, G2C%d((%s).xy), (%s).w G2A%d), (%s).xy), (%s).xyz)", unit, unit, unit, unit, coord, coord, unit, coord, coord);
    else if (how == 1) snprintf(t, sizeof t, "G2X%d(G2E%d(texture2D(s%d, G2C%d((%s).xy / (%s).w) G2B%d), (%s).xy / (%s).w), (%s).xyz / (%s).w)", unit, unit, unit, unit, coord, coord, unit, coord, coord, coord, coord);
    else if (how == 2) snprintf(t, sizeof t, "G2X%d(G2E%d(texture2D(s%d, G2C%d((%s).xy), (%s).w G2A%d), (%s).xy), (%s).xyz)", unit, unit, unit, unit, coord, coord, unit, coord, coord);
    else snprintf(t, sizeof t, "G2X%d(G2E%d(texture2D(s%d, G2C%d((%s).xy) G2B%d), (%s).xy), (%s).xyz)", unit, unit, unit, unit, coord, unit, coord, coord);
    snprintf(o, n, "G2S%d(%s)", unit, t);
}

static const char *CMP_T[7] = { "", ">", "==", ">=", "<", "!=", "<=" };

/* a comparison per component, 1s and 0s */
static void cmp_vec(int cmp, const char *a, const char *b, char *o, size_t n)
{
    switch (cmp) {
    case 1: snprintf(o, n, "vec4(1.0) - step(%s, %s)", a, b); break;            /* a > b */
    case 2: snprintf(o, n, "step(%s, %s) * step(%s, %s)", a, b, b, a); break;
    case 3: snprintf(o, n, "step(%s, %s)", b, a); break;
    case 4: snprintf(o, n, "vec4(1.0) - step(%s, %s)", b, a); break;
    case 5: snprintf(o, n, "vec4(1.0) - step(%s, %s) * step(%s, %s)", a, b, b, a); break;
    default: snprintf(o, n, "step(%s, %s)", a, b); break;
    }
}

/* a loop's count, start or step: a number if defi gave it, else the macro of what SetVertexShaderConstantI gave */
static void int_text(sm *m, int r, int comp, char *o, size_t n)
{
    if (m->def_i[r]) { snprintf(o, n, "%d", m->defi[r][comp]); return; }
    m->s->loop_i |= 1u << r;
    snprintf(o, n, "G2I%d%c", r, "XYZ"[comp]);
}

/* t<n> of ps 1.x: the texture coordinate, the texture stage's sample */
static int ps1_stage(sm *m, const ins *i) { int n = reg_num(i->d) & 7; m->in_t[n] = 1; return n; }

/* one row of a texm3x matrix: the coordinates of its stage with the normal. The rows before it are in g2m<stage - 1> .. */
static void m3x_row(sm *m, const char *a, int stage) { d3sb_printf(&m->body, "    float g2m%d = dot(v_t%d.xyz, (%s).xyz);\n", stage, stage, a); }

static void emit_ins(sm *m, const ins *i)
{
    char a[120], b[120], c[120], e[512], rn[32];
    int op = i->op, k;
    d3sb *o = &m->body;
    a[0] = b[0] = c[0] = 0;
    if (!m->ps && (op == O_MOV || op == O_MOVA) && reg_type(i->d) == R_ADDR) {     /* vs 1.x floors, mova rounds */
        a0_load(m, i->d, i->s[0], i->srel[0], op == O_MOVA);
        for (k = 0; k < 4; k++) {
            if (!(i->d >> (16 + k) & 1) || m->a0_reg[k] != A0_VERTEX) continue;
            src_text(m, i->s[0], i->srel[0], 0, a, sizeof a);
            m->a0_var[k] = 1;
            d3sb_printf(o, "    a0%c = int(floor((%s).%c%s));\n", "xyzw"[k], a, "xyzw"[k], op == O_MOVA ? " + 0.5" : "");
        }
        return;
    }
    if (i->ns > 0 && op != O_TEX && op != O_REP && op != O_LOOP) src_text(m, i->s[0], i->srel[0], 0, a, sizeof a);
    if (i->ns > 1 && op != O_TEX && op != O_LOOP && op != O_TEXLDL && op != O_TEXLDD && !(op >= O_M4x4 && op <= O_M3x2)) src_text(m, i->s[1], i->srel[1], 0, b, sizeof b);
    if (i->ns > 2) src_text(m, i->s[2], i->srel[2], 0, c, sizeof c);
    if (op_is_scalar_src(op)) { size_t l = strlen(a); if (l + 4 < sizeof a) { char t[120]; snprintf(t, sizeof t, "(%s).w", a); strcpy(a, t); } }
    switch (op) {
    case O_NOP: break;
    case O_MOV: assign(m, i, a); break;
    case O_ADD: snprintf(e, sizeof e, "%s + %s", a, b); assign(m, i, e); break;
    case O_SUB: snprintf(e, sizeof e, "%s - %s", a, b); assign(m, i, e); break;
    case O_MUL: snprintf(e, sizeof e, "%s * %s", a, b); assign(m, i, e); break;
    case O_MAD: snprintf(e, sizeof e, "%s * %s + %s", a, b, c); assign(m, i, e); break;
    case O_LRP: snprintf(e, sizeof e, "mix(%s, %s, %s)", c, b, a); assign(m, i, e); break;
    case O_RCP: snprintf(e, sizeof e, "vec4(1.0 / %s)", a); assign(m, i, e); break;
    case O_RSQ: snprintf(e, sizeof e, "vec4(inversesqrt(abs(%s)))", a); assign(m, i, e); break;
    case O_EXP: snprintf(e, sizeof e, "vec4(exp2(%s))", a); assign(m, i, e); break;
    case O_LOG: snprintf(e, sizeof e, "vec4(log2(abs(%s)))", a); assign(m, i, e); break;
    case O_POW: { char t[120]; snprintf(t, sizeof t, "(%s).w", b); snprintf(e, sizeof e, "vec4(pow(abs(%s), %s))", a, t); assign(m, i, e); break; }
    case O_DP3: snprintf(e, sizeof e, "vec4(dot((%s).xyz, (%s).xyz))", a, b); assign(m, i, e); break;
    case O_DP4: snprintf(e, sizeof e, "vec4(dot(%s, %s))", a, b); assign(m, i, e); break;
    case O_MIN: snprintf(e, sizeof e, "min(%s, %s)", a, b); assign(m, i, e); break;
    case O_MAX: snprintf(e, sizeof e, "max(%s, %s)", a, b); assign(m, i, e); break;
    case O_SLT: snprintf(e, sizeof e, "vec4(1.0) - step(%s, %s)", b, a); assign(m, i, e); break;
    case O_SGE: snprintf(e, sizeof e, "step(%s, %s)", b, a); assign(m, i, e); break;
    case O_FRC: snprintf(e, sizeof e, "fract(%s)", a); assign(m, i, e); break;
    case O_ABS: snprintf(e, sizeof e, "abs(%s)", a); assign(m, i, e); break;
    case O_SGN: snprintf(e, sizeof e, "sign(%s)", a); assign(m, i, e); break;
    case O_NRM: snprintf(e, sizeof e, "(%s) * inversesqrt(dot((%s).xyz, (%s).xyz))", a, a, a); assign(m, i, e); break;
    case O_CRS: snprintf(e, sizeof e, "vec4(cross((%s).xyz, (%s).xyz), 0.0)", a, b); assign(m, i, e); break;
    case O_LIT: snprintf(e, sizeof e, "vec4(1.0, max((%s).x, 0.0), (%s).x > 0.0 ? pow(max((%s).y, 0.0), clamp((%s).w, -128.0, 128.0)) : 0.0, 1.0)", a, a, a, a); assign(m, i, e); break;
    case O_DST: snprintf(e, sizeof e, "vec4(1.0, (%s).y * (%s).y, (%s).z, (%s).w)", a, b, a, b); assign(m, i, e); break;
    case O_EXPP: snprintf(e, sizeof e, "vec4(exp2(floor(%s)), fract(%s), exp2(%s), 1.0)", a, a, a); assign(m, i, e); break;
    case O_LOGP: snprintf(e, sizeof e, "vec4(floor(log2(abs(%s))), abs(%s) / exp2(floor(log2(abs(%s)))), log2(abs(%s)), 1.0)", a, a, a, a); assign(m, i, e); break;
    case O_SINCOS: snprintf(e, sizeof e, "vec4(cos(%s), sin(%s), 0.0, 0.0)", a, a); assign(m, i, e); break;
    case O_CMP: snprintf(e, sizeof e, "mix(%s, %s, step(vec4(0.0), %s))", c, b, a); assign(m, i, e); break;
    case O_CND: snprintf(e, sizeof e, "mix(%s, %s, vec4(1.0) - step(%s, vec4(0.5)))", c, b, a); assign(m, i, e); break;
    case O_DP2ADD: snprintf(e, sizeof e, "vec4(dot((%s).xy, (%s).xy) + (%s).x)", a, b, c); assign(m, i, e); break;
    case O_DSX: snprintf(e, sizeof e, "dFdx(%s)", a); assign(m, i, e); break;
    case O_DSY: snprintf(e, sizeof e, "dFdy(%s)", a); assign(m, i, e); break;
    case O_M4x4: case O_M4x3: case O_M3x4: case O_M3x3: case O_M3x2: {
        int rows = op == O_M4x4 || op == O_M3x4 ? 4 : op == O_M3x2 ? 2 : 3, vec4_in = op == O_M4x4 || op == O_M4x3;
        char t[4][120];
        for (k = 0; k < rows; k++) src_text(m, i->s[1], i->srel[1], k, t[k], sizeof t[k]);
        if (vec4_in) snprintf(e, sizeof e, "vec4(dot(%s, %s), dot(%s, %s), %s%s%s%s", a, t[0], a, t[1], "", "", "", "");
        else snprintf(e, sizeof e, "vec4(dot((%s).xyz, (%s).xyz), dot((%s).xyz, (%s).xyz), ", a, t[0], a, t[1]);
        {
            size_t n = strlen(e);
            if (rows > 2) n += (size_t)snprintf(e + n, sizeof e - n, vec4_in ? "dot(%s, %s), " : "dot((%s).xyz, (%s).xyz), ", a, t[2]);
            else n += (size_t)snprintf(e + n, sizeof e - n, "0.0, ");
            if (rows > 3) snprintf(e + n, sizeof e - n, vec4_in ? "dot(%s, %s))" : "dot((%s).xyz, (%s).xyz))", a, t[3]);
            else snprintf(e + n, sizeof e - n, "0.0)");
        }
        assign(m, i, e);
        break;
    }
    case O_TEXCOORD:
        if (m->major == 1 && m->minor == 4) { assign(m, i, a); break; }
        m->in_t[reg_num(i->d) & 7] = 1;
        snprintf(e, sizeof e, "clamp(v_t%d, 0.0, 1.0)", reg_num(i->d) & 7);
        assign(m, i, e);
        break;
    case O_TEXKILL: {
        char d[48];
        DWORD wm = i->d >> 16 & 7;
        reg_text(m, i->d, d, sizeof d, 0);
        d3sb_printf(o, "    if (");
        for (k = 0; k < 3; k++) if (wm >> k & 1) d3sb_printf(o, "%s%s.%c < 0.0", (wm & ((1u << k) - 1)) ? " || " : "", d, "xyz"[k]);
        d3sb_printf(o, ") discard;\n");
        break;
    }
    case O_TEX: {
        char co[120], tx[1000];
        int unit, how = (int)(i->tok >> 16 & 3);                /* texldp 1, texldb 2 */
        if (m->major >= 2) { src_text(m, i->s[0], i->srel[0], 0, co, sizeof co); unit = reg_num(i->s[1]); }
        else if (m->minor == 4) { src_text(m, i->s[0], i->srel[0], 0, co, sizeof co); unit = reg_num(i->d); }
        else { unit = reg_num(i->d); m->in_t[unit & 7] = 1; snprintf(co, sizeof co, "v_t%d", unit & 7); }
        if (!m->ps) { fail(m, "texture sampling in a vertex shader: MaxVertexTextures is 0"); break; }
        tex_expr(m, unit, co, how, tx, sizeof tx);
        assign(m, i, tx);
        break;
    }
    case O_TEXLDL: {
        char co[120], tx[1000];
        src_text(m, i->s[0], i->srel[0], 0, co, sizeof co);
        if (!m->ps) { fail(m, "texture sampling in a vertex shader: MaxVertexTextures is 0"); break; }
        tex_expr(m, reg_num(i->s[1]), co, 3, tx, sizeof tx);
        assign(m, i, tx);
        break;
    }
    case O_TEXLDD: {                                            /* the level the gradients ask for, as the TMU would work it out from them */
        char co[120], dx[120], dy[120], tx[1000];
        int unit = reg_num(i->s[1]) & 7;
        src_text(m, i->s[0], i->srel[0], 0, co, sizeof co);
        src_text(m, i->s[2], i->srel[2], 0, dx, sizeof dx);
        src_text(m, i->s[3], i->srel[3], 0, dy, sizeof dy);
        if (!m->ps) { fail(m, "texture sampling in a vertex shader: MaxVertexTextures is 0"); break; }
        m->tsize[unit] = 1;
        if (m->samp[unit] == 4) d3sb_printf(o, "    {\n    vec3 g2_dx = (%s).xyz * pz%d.xyz;\n    vec3 g2_dy = (%s).xyz * pz%d.xyz;\n", dx, unit, dy, unit);
        else d3sb_printf(o, "    {\n    vec2 g2_dx = (%s).xy * pz%d.xy;\n    vec2 g2_dy = (%s).xy * pz%d.xy;\n", dx, unit, dy, unit);
        d3sb_printf(o, "    vec4 g2_c = vec4((%s).xyz, 0.5 * log2(max(max(dot(g2_dx, g2_dx), dot(g2_dy, g2_dy)), 1.0e-6)));\n", co);
        tex_expr(m, unit, "g2_c", 3, tx, sizeof tx);
        assign(m, i, tx);
        d3sb_printf(o, "    }\n");
        break;
    }
    case O_SETP: cmp_vec((int)(i->tok >> 16 & 7), a, b, e, sizeof e); assign(m, i, e); break;
    case O_BREAKP: d3sb_printf(o, "    if ((%s).x != 0.0) break;\n", a); break;
    case O_TEXBEM: case O_TEXBEML: {                            /* the coordinates of this stage, moved by the stage before's texel through this stage's matrix */
        char tx[1800];
        int n = ps1_stage(m, i);
        m->bem[n] = 1;
        snprintf(e, sizeof e, "vec4(v_t%d.x + pm%d.x * (%s).x + pm%d.z * (%s).y, v_t%d.y + pm%d.y * (%s).x + pm%d.w * (%s).y, 0.0, 1.0)", n, n, a, n, a, n, n, a, n, a);
        tex_expr(m, n, e, 0, tx, sizeof tx);
        if (op == O_TEXBEML) {
            char l[300];
            m->lum[n] = 1;
            snprintf(l, sizeof l, "vec4(g2_b.rgb * clamp((%s).z * pl%d.x + pl%d.y, 0.0, 1.0), g2_b.a)", a, n, n);
            d3sb_printf(o, "    {\n    vec4 g2_b = %s;\n", tx);
            assign(m, i, l);
            d3sb_printf(o, "    }\n");
            break;
        }
        assign(m, i, tx);
        break;
    }
    case O_BEM: {                                               /* ps_1_4: the same sum, the first source for the coordinates */
        int n = reg_num(i->d) & 7;
        m->bem[n] = 1;
        snprintf(e, sizeof e, "vec4((%s).x + pm%d.x * (%s).x + pm%d.z * (%s).y, (%s).y + pm%d.y * (%s).x + pm%d.w * (%s).y, 0.0, 0.0)", a, n, b, n, b, a, n, b, n, b);
        assign(m, i, e);
        break;
    }
    case O_TEXREG2AR: case O_TEXREG2GB: case O_TEXREG2RGB: {
        char tx[1800];
        int n = reg_num(i->d) & 7;
        if (op == O_TEXREG2AR) snprintf(e, sizeof e, "vec4((%s).w, (%s).x, 0.0, 1.0)", a, a);
        else if (op == O_TEXREG2GB) snprintf(e, sizeof e, "vec4((%s).y, (%s).z, 0.0, 1.0)", a, a);
        else snprintf(e, sizeof e, "vec4((%s).xyz, 1.0)", a);
        tex_expr(m, n, e, 0, tx, sizeof tx);
        assign(m, i, tx);
        break;
    }
    case O_TEXDP3: case O_TEXDP3TEX: {
        char tx[1800];
        int n = ps1_stage(m, i);
        snprintf(e, sizeof e, "vec4(dot(v_t%d.xyz, (%s).xyz), 0.0, 0.0, 1.0)", n, a);
        if (op == O_TEXDP3) { snprintf(e, sizeof e, "vec4(dot(v_t%d.xyz, (%s).xyz))", n, a); assign(m, i, e); break; }
        tex_expr(m, n, e, 0, tx, sizeof tx);
        assign(m, i, tx);
        break;
    }
    case O_TEXM3x2PAD: case O_TEXM3x3PAD: m3x_row(m, a, ps1_stage(m, i)); break;
    case O_TEXM3x2TEX: {
        char tx[1800];
        int n = ps1_stage(m, i);
        m3x_row(m, a, n);
        snprintf(e, sizeof e, "vec4(g2m%d, g2m%d, 0.0, 1.0)", (n - 1) & 7, n);
        tex_expr(m, n, e, 0, tx, sizeof tx);
        assign(m, i, tx);
        break;
    }
    case O_TEXM3x2DEPTH: {                                      /* z / w, 1 where w is 0 */
        int n = ps1_stage(m, i);
        m3x_row(m, a, n);
        m->depth_out = 1;
        d3sb_printf(o, "    g2_depth = vec4(g2m%d == 0.0 ? 1.0 : g2m%d / g2m%d);\n", n, (n - 1) & 7, n);
        break;
    }
    case O_TEXM3x3: case O_TEXM3x3TEX: case O_TEXM3x3SPEC: case O_TEXM3x3VSPEC: {
        char tx[1800];
        int n = ps1_stage(m, i);
        m3x_row(m, a, n);
        d3sb_printf(o, "    {\n    vec3 g2_n = vec3(g2m%d, g2m%d, g2m%d);\n", (n - 2) & 7, (n - 1) & 7, n);
        if (op == O_TEXM3x3) { assign(m, i, "vec4(g2_n, 1.0)"); d3sb_printf(o, "    }\n"); break; }
        if (op == O_TEXM3x3TEX) snprintf(e, sizeof e, "vec4(g2_n, 1.0)");
        else {                                                  /* the reflection of the eye vector: a constant, or the fourth components of the three coordinates */
            if (op == O_TEXM3x3SPEC) d3sb_printf(o, "    vec3 g2_e = (%s).xyz;\n", b);
            else { m->in_t[(n - 2) & 7] = m->in_t[(n - 1) & 7] = 1; d3sb_printf(o, "    vec3 g2_e = vec3(v_t%d.w, v_t%d.w, v_t%d.w);\n", (n - 2) & 7, (n - 1) & 7, n); }
            snprintf(e, sizeof e, "vec4(2.0 * dot(g2_n, g2_e) / dot(g2_n, g2_n) * g2_n - g2_e, 1.0)");
        }
        tex_expr(m, n, e, 0, tx, sizeof tx);
        assign(m, i, tx);
        d3sb_printf(o, "    }\n");
        break;
    }
    case O_TEXDEPTH: {                                          /* ps_1_4: r5.r / r5.g */
        char d[48];
        reg_text(m, i->d, d, sizeof d, 0);
        m->depth_out = 1;
        d3sb_printf(o, "    g2_depth = vec4(%s.y == 0.0 ? 1.0 : clamp(%s.x / %s.y, 0.0, 1.0));\n", d, d, d);
        break;
    }
    case O_IF: d3sb_printf(o, "    if (%s.x != 0.0) {\n", a); break;
    case O_IFC: d3sb_printf(o, "    if ((%s).x %s (%s).x) {\n", a, CMP_T[i->tok >> 16 & 7], b); break;
    case O_ELSE: d3sb_printf(o, "    } else {\n"); break;
    case O_ENDIF: d3sb_printf(o, "    }\n"); break;
    case O_BREAK: d3sb_printf(o, "    break;\n"); break;
    case O_BREAKC: d3sb_printf(o, "    if ((%s).x %s (%s).x) break;\n", a, CMP_T[i->tok >> 16 & 7], b); break;
    case O_REP: {
        int r = reg_num(i->s[0]) & 15;
        char n[16];
        int_text(m, r, 0, n, sizeof n);
        d3sb_printf(o, "    for (int rep%d = 0; rep%d < %s; rep%d++) {\n", m->reps, m->reps, n, m->reps);
        m->reps++;
        break;
    }
    case O_ENDREP: case O_ENDLOOP: d3sb_printf(o, "    }\n"); break;
    case O_LOOP: {
        int r = reg_num(i->s[1]) & 15;
        char n[16], st[16], by[16];
        int_text(m, r, 0, n, sizeof n); int_text(m, r, 1, st, sizeof st); int_text(m, r, 2, by, sizeof by);
        d3sb_printf(o, "    for (int aL = %s, n_ = 0; n_ < %s; n_++, aL += %s) {\n", st, n, by);
        break;
    }
    default: snprintf(rn, sizeof rn, "opcode %d", op); fail(m, rn); break;
    }
}

/* the code from p: main, or a subroutine written out at its call, up to its ret. Anything but a call goes to emit_ins */
static void walk(sm *m, const DWORD *p, int depth)
{
    int nest = 0;
    while (p < m->end && (*p & 0xFFFF) != O_END && !m->err[0]) {
        DWORD t = *p;
        int op = (int)(t & 0xFFFF);
        ins x;
        if (op == O_COMMENT) { p += 1 + (t >> 16); continue; }
        if (op == O_PHASE) { p++; continue; }
        if (op == O_DCL) { p += 3; continue; }
        if (op == O_DEF || op == O_DEFI) { p += 6; continue; }
        if (op == O_DEFB) { p += 3; continue; }
        memset(&x, 0, sizeof x);
        if (read_ins(m, &p, &x) < 0) return;
        switch (x.op) {
        case O_LABEL: return;
        case O_RET:
            if (nest) fail(m, "a ret inside a branch or a loop");
            return;
        case O_CALL: case O_CALLNZ: {
            int l = reg_num(x.s[0]) & 31;
            if (!m->label[l]) { fail(m, "a call of a label that is not there"); return; }
            if (depth >= 8) { fail(m, "subroutines nested deeper than the stack, or calling themselves"); return; }
            if (x.op == O_CALLNZ) { char c[120]; src_text(m, x.s[1], x.srel[1], 0, c, sizeof c); d3sb_printf(&m->body, "    if ((%s).x != 0.0) {\n", c); }
            walk(m, m->label[l], depth + 1);
            if (x.op == O_CALLNZ) d3sb_printf(&m->body, "    }\n");
            continue;
        }
        case O_IF: case O_IFC: case O_LOOP: case O_REP: nest++; break;
        case O_ENDIF: case O_ENDLOOP: case O_ENDREP: nest--; break;
        default: break;
        }
        if (!m->ps && m->ins_n++ == m->zr_at) {                 /* the position's z and w are dot products of this vector, which holds that value here (pos_scan) */
            char v[120];
            src_text(m, m->zr_x, 0, 0, v, sizeof v);
            d3sb_printf(&m->body, "    G2_ZR(%s)\n", v);
        }
        emit_ins(m, &x);
    }
}

/* Where a vertex shader's position gets its z and w (depth kept reversed, d3d9_ff.c). A far vertex has z just under w and both are FP24 on board A, so w - z worked out there is their
 * rounding. When oPos.z = dot(X, c[zk]) and oPos.w = dot(X, c[wk]) of one X — m4x4 oPos, v0, c0, or through a temporary and a mov, what games write — the code gets G2_ZR(X) at a place where X
 * holds that value, and d3d9_ff.c makes it z' = dot(X, c[wk] - c[zk]) with the difference worked out on the PC. Anything else leaves pos_zk < 0: z' is then w - z in the program, no finer than Direct3D's own count.
 * The instructions are counted as walk() counts them, which holds while there is no flow control: with any, there is no pattern */
typedef struct ptag { int at; DWORD x; int k; } ptag;       /* the instruction, its vector operand, the constant register; at < 0: not such a dot product */
#define POS_MAXI 4096
static int plain_const(const sm *m, DWORD p, int rel, int rows)
{
    int r = reg_num(p), k;
    if (reg_type(p) != R_CONST || rel || (p >> 24 & 15) || (p >> 16 & 255) != 0xE4 || r + rows > VS_CONSTS) return 0;
    for (k = 0; k < rows; k++) if (m->def_c[r + k]) return 0;
    return 1;
}
static int plain_vec(DWORD p, int rel) { return !rel && (reg_type(p) == R_INPUT || (reg_type(p) == R_TEMP && reg_num(p) < 32)); }

typedef struct pscan { ptag tg[32][2], out[2]; short wd[POS_MAXI]; int n, flow; } pscan;
static void pos_step(sm *m, pscan *q, const ins *x)
{
    ptag t[4];
    int c, ty, r, n = q->n++;
    DWORD wm;
    if (n >= POS_MAXI) { q->flow = 1; return; }
    q->wd[n] = -1;
    switch (x->op) {
    case O_CALL: case O_CALLNZ: case O_LOOP: case O_RET: case O_ENDLOOP: case O_LABEL: case O_REP: case O_ENDREP: case O_IF: case O_IFC: case O_ELSE: case O_ENDIF: case O_BREAK: case O_BREAKC:
    case O_BREAKP: case O_SETP: q->flow = 1; return;
    default: break;
    }
    if (x->tok & INS_PREDICATED) q->flow = 1;
    if (!x->has_d) return;
    ty = reg_type(x->d); r = reg_num(x->d); wm = x->d >> 16 & 15;
    for (c = 0; c < 4; c++) t[c].at = -1;
    if (!(x->d >> 20 & 0xFF)) {                                 /* no saturation, no shift */
        if ((x->op == O_M4x4 || x->op == O_M4x3) && plain_vec(x->s[0], x->srel[0]) && plain_const(m, x->s[1], x->srel[1], x->op == O_M4x4 ? 4 : 3)) {
            for (c = 0; c < (x->op == O_M4x4 ? 4 : 3); c++) { t[c].at = n; t[c].x = x->s[0]; t[c].k = reg_num(x->s[1]) + c; }
        } else if (x->op == O_DP4) {
            int v = plain_vec(x->s[0], x->srel[0]) && plain_const(m, x->s[1], x->srel[1], 1) ? 0 : plain_vec(x->s[1], x->srel[1]) && plain_const(m, x->s[0], x->srel[0], 1) ? 1 : -1;
            if (v >= 0) for (c = 0; c < 4; c++) { t[c].at = n; t[c].x = x->s[v]; t[c].k = reg_num(x->s[1 - v]); }
        } else if (x->op == O_MOV && reg_type(x->s[0]) == R_TEMP && reg_num(x->s[0]) < 32 && !x->srel[0] && !(x->s[0] >> 24 & 15)) {
            for (c = 2; c < 4; c++) { int sc = (int)(x->s[0] >> (16 + 2 * c) & 3); if (sc >= 2) t[c] = q->tg[reg_num(x->s[0])][sc - 2]; }
        }
    }
    if (ty == R_TEMP && r < 32) {
        q->wd[n] = (short)r;
        for (c = 2; c < 4; c++) if (wm >> c & 1) q->tg[r][c - 2] = t[c];
    } else if ((ty == R_RASTOUT && r == 0) || (ty == R_OUTPUT && m->major >= 3 && r < 12 && m->out_o_usage[r] == D3DDECLUSAGE_POSITION)) {
        for (c = 2; c < 4; c++) if (wm >> c & 1) q->out[c - 2] = t[c];
    }
}

static void pos_done(sm *m, const pscan *q)
{
    const ptag *z = &q->out[0], *w = &q->out[1];
    int lo, hi, i;
    m->zr_at = -1; m->s->pos_zk = m->s->pos_wk = -1;
    if (q->flow || m->a0_vertex || z->at < 0 || w->at < 0 || z->x != w->x) return;
    lo = z->at < w->at ? z->at : w->at; hi = z->at < w->at ? w->at : z->at;
    if (reg_type(z->x) == R_TEMP) for (i = lo; i < hi; i++) if (q->wd[i] == reg_num(z->x)) return;     /* the vector changed between the two */
    m->zr_at = hi; m->zr_x = z->x;
    m->s->pos_zk = z->k; m->s->pos_wk = w->k;
}

int d3_sm_parse(d3shader *s)
{
    sm *m = (sm *)calloc(1, sizeof *m);
    const DWORD *p;
    d3sb head;
    int i, rc = 0;
    pscan *q = (pscan *)calloc(1, sizeof *q);
    if (!m || !q) { free(m); free(q); snprintf(s->error, sizeof s->error, "out of memory"); return -1; }
    for (i = 0; i < 32; i++) q->tg[i][0].at = q->tg[i][1].at = -1;
    q->out[0].at = q->out[1].at = -1;
    m->zr_at = -1; s->pos_zk = s->pos_wk = -1;
    memset(&head, 0, sizeof head);
    m->s = s; m->t = s->tok; m->end = s->tok + s->ntok;
    m->major = (int)(s->tok[0] >> 8 & 255); m->minor = (int)(s->tok[0] & 255); m->ps = s->ps;
    s->major = m->major; s->minor = m->minor;
    for (i = 0; i < 4; i++) m->a0_reg[i] = -1;
    /* first pass: declarations and constants */
    for (p = s->tok + 1; p < m->end && (*p & 0xFFFF) != O_END; ) {
        DWORD t = *p;
        int op = (int)(t & 0xFFFF);
        if (op == O_COMMENT) { p += 1 + (t >> 16); continue; }
        if (op == O_PHASE) { p++; continue; }
        if (op == O_DCL) {
            DWORD u = p[1], r = p[2];
            int ty = reg_type(r), n = reg_num(r), usage = (int)(u & 31), idx = (int)(u >> 16 & 15);
            if (ty == R_SAMPLER) {
                int kind = (int)(u >> 27 & 15);                 /* D3DSTT_2D 2, CUBE 3, VOLUME 4 */
                if (n < 16) m->samp[n] = (uint8_t)(kind == 3 || kind == 4 ? kind : 2);
            } else if (ty == R_INPUT) {
                if (!s->ps && s->nin < 32) { s->in[s->nin].usage = usage; s->in[s->nin].index = idx; snprintf(s->in[s->nin].name, sizeof s->in[0].name, "%d", n); s->nin++; }
                if (s->ps && n < 16) { m->in_usage[n] = (uint8_t)usage; m->in_index[n] = (uint8_t)idx; m->in_dcl[n] = 1; }
                if (s->ps && usage == D3DDECLUSAGE_TEXCOORD && idx < 8) { DWORD w = r >> 16 & 15; s->tcomps[idx] = (uint8_t)(w & 8 ? 4 : w & 4 ? 3 : w & 2 ? 2 : 1); }
            } else if (s->ps && ty == R_ADDR && n < 8) {        /* ps_2_0: dcl t<n> */
                DWORD w = r >> 16 & 15;
                s->tcomps[n] = (uint8_t)(w & 8 ? 4 : w & 4 ? 3 : w & 2 ? 2 : 1);
            } else if (ty == R_OUTPUT && n < 12) {
                m->out_o_usage[n] = (uint8_t)usage; m->out_o_index[n] = (uint8_t)idx;
            }
            p += 3;
            continue;
        }
        if (op == O_DEF) {
            int r = reg_num(p[1]);
            if (r < VS_CONSTS) {
                float f[4];
                memcpy(f, p + 2, 16);
                m->def_c[r] = 1;
                d3sb_printf(&m->body, "    vec4 dc%d = vec4(%.9g, %.9g, %.9g, %.9g);\n", r, f[0], f[1], f[2], f[3]);
                if (!s->defc) s->defc = (float *)calloc(VS_CONSTS * 4, sizeof(float));
                if (s->defc) { memcpy(s->defc + 4 * r, f, 16); s->def_mask[r / 32] |= 1u << (r % 32); }
            }
            p += 6;
            continue;
        }
        if (op == O_DEFI) { int r = reg_num(p[1]) & 15; memcpy(m->defi[r], p + 2, 16); m->def_i[r] = 1; p += 6; continue; }
        if (op == O_DEFB) { m->def_b[reg_num(p[1]) & 15] = p[2] ? 2 : 1; p += 3; continue; }
        {
            ins x;
            memset(&x, 0, sizeof x);
            if (read_ins(m, &p, &x) < 0) break;
            if (x.op == O_LABEL) m->label[reg_num(x.s[0]) & 31] = p;
            if (!s->ps && (x.op == O_MOV || x.op == O_MOVA) && reg_type(x.d) == R_ADDR && !(reg_type(x.s[0]) == R_CONST && !x.srel[0] && !(x.s[0] >> 24 & 15))) m->a0_vertex = 1;
            if (!s->ps) for (i = 0; i < x.ns; i++) if (x.srel[i] == REL_LOOP) m->a0_vertex = 1;
            if (!s->ps) pos_step(m, q, &x);
        }
    }
    if (!s->ps && !m->err[0]) pos_done(m, q);
    free(q);
    /* second pass: code */
    if (!m->err[0]) {
        d3sb code;
        memset(&code, 0, sizeof code);
        { d3sb t = m->body; m->body = code; code = t; }          /* defs go first */
        walk(m, s->tok + 1, 0);
        /* assemble: declarations, then temporaries, then code */
        if (s->ps && m->major == 1) m->temps[0] = 1;            /* r0 is the result of ps 1.x */
        for (i = 0; i < 32; i++) if (m->temps[i]) d3sb_printf(&head, "    vec4 r%d = vec4(0.0);\n", i);
        for (i = 0; i < 8; i++) if (m->tex_tmp[i]) d3sb_printf(&head, "    vec4 t%d = vec4(0.0);\n", i);
        for (i = 0; i < 4; i++) if (m->a0_var[i]) d3sb_printf(&head, "    int a0%c = 0;\n", "xyzw"[i]);
        if (m->pred) d3sb_printf(&head, "    vec4 g2_p = vec4(0.0);\n");
        if (code.p) d3sb_printf(&head, "%s", code.p);
        if (!s->ps) {
            for (i = 0; i < s->nin; i++) {
                int n = atoi(s->in[i].name);
                d3sb_printf(&m->decl, "attribute vec4 a_u%d_%d;\n", s->in[i].usage, s->in[i].index);
                if (n < 32) d3sb_printf(&head, "    vec4 v%d = a_u%d_%d SW_u%d_%d;\n", n, s->in[i].usage, s->in[i].index, s->in[i].usage, s->in[i].index);
            }
            for (i = 0; i < 2; i++) if (m->out_c[i]) { d3sb_printf(&m->decl, "varying vec4 v_c%d;\n", i); if (m->out_c_mask[i] != 15) d3sb_printf(&head, "    v_c%d = vec4(0.0, 0.0, 0.0, 1.0);\n", i); }
            for (i = 0; i < 10; i++) if (m->out_t[i]) { d3sb_printf(&m->decl, "varying vec4 v_t%d;\n", i); if (m->out_t_mask[i] != 15) d3sb_printf(&head, "    v_t%d = vec4(0.0, 0.0, 0.0, 1.0);\n", i); }
            for (i = 0; i < 8; i++) if (m->out_t[i]) s->tcomps[i] = (uint8_t)(m->out_t_mask[i] & 8 ? 4 : m->out_t_mask[i] & 4 ? 3 : m->out_t_mask[i] & 2 ? 2 : 1);
            if (m->major >= 3) {
                for (i = 0; i < 12; i++) {
                    int u = m->out_o_usage[i], x = m->out_o_index[i];
                    if (!m->out_o[i] || u == D3DDECLUSAGE_POSITION || u == D3DDECLUSAGE_FOG || u == D3DDECLUSAGE_PSIZE) continue;
                    if (u == D3DDECLUSAGE_COLOR) d3sb_printf(&m->decl, "varying vec4 v_c%d;\n", x);
                    else if (u == D3DDECLUSAGE_TEXCOORD) d3sb_printf(&m->decl, "varying vec4 v_t%d;\n", x);
                    else d3sb_printf(&m->decl, "varying vec4 v_u%d_%d;\n", u, x);
                }
            }
            if (m->out_fog) { d3sb_printf(&m->decl, "varying float v_fog;\n"); d3sb_printf(&head, "    vec4 v_fogv = vec4(1.0);\n"); }
            d3sb_printf(&head, "    vec4 v_psz = vec4(G2_PSIZE);\n");
            for (i = 0; i < VS_CONSTS; i++) if (m->used_c[i]) d3sb_printf(&m->decl, "uniform vec4 vc%d;\n", i);
            for (i = 0; i < s->nrel; i++) d3sb_printf(&m->decl, "uniform vec4 vr%d;\n", i);
            if (m->a0_vertex) d3sb_printf(&m->decl, "uniform vec4 vca[%d];\n", VCA_N);
            for (i = 0; i < 16; i++) { if (m->used_i[i]) d3sb_printf(&m->decl, "uniform vec4 vi%d;\n", i); if (m->used_b[i]) d3sb_printf(&m->decl, "uniform vec4 vb%d;\n", i); }
        } else {
            for (i = 0; i < 2; i++) if (m->in_c[i]) { d3sb_printf(&m->decl, "varying vec4 v_c%d;\n", i); if (s->nin < 32) { s->in[s->nin].usage = D3DDECLUSAGE_COLOR; s->in[s->nin].index = i; s->nin++; } }
            for (i = 0; i < 10; i++) if (m->in_t[i]) { d3sb_printf(&m->decl, "varying vec4 v_t%d;\n", i); if (s->nin < 32) { s->in[s->nin].usage = D3DDECLUSAGE_TEXCOORD; s->in[s->nin].index = i; s->nin++; } }
            if (m->major >= 3) {
                for (i = 0; i < 16; i++) {
                    char n[24];
                    if (!m->in_dcl[i] || !m->inputs[i]) continue;
                    in_name(m, i, n, sizeof n);
                    d3sb_printf(&m->decl, "varying vec4 %s;\n", n);
                    if (s->nin < 32) { s->in[s->nin].usage = m->in_usage[i]; s->in[s->nin].index = m->in_index[i]; s->nin++; }
                }
            }
            for (i = 0; i < 16; i++) {
                if (i < 8) s->samp_kind[i] = m->samp[i];
                if (m->samp[i] > 1) d3sb_printf(&m->decl, "uniform %s s%d;\n", m->samp[i] == 3 ? "samplerCube" : m->samp[i] == 4 ? "sampler3D" : "sampler2D", i);
            }
            for (i = 0; i < PS_CONSTS; i++) if (m->used_c[i]) d3sb_printf(&m->decl, "uniform vec4 pc%d;\n", i);
            for (i = 0; i < 16; i++) { if (m->used_i[i]) d3sb_printf(&m->decl, "uniform vec4 pi%d;\n", i); if (m->used_b[i]) d3sb_printf(&m->decl, "uniform vec4 pb%d;\n", i); }
            for (i = 0; i < 8; i++) {
                if (m->bem[i]) d3sb_printf(&m->decl, "uniform vec4 pm%d;\n", i);
                if (m->lum[i]) d3sb_printf(&m->decl, "uniform vec4 pl%d;\n", i);
                if (m->tsize[i]) d3sb_printf(&m->decl, "uniform vec4 pz%d;\n", i);
            }
            d3sb_printf(&head, "    vec4 g2_out = vec4(0.0);\n");
            if (m->depth_out) d3sb_printf(&head, "    vec4 g2_depth = vec4(0.0);\n");
            if (m->out_extra) d3sb_printf(&head, "    vec4 g2_drop = vec4(0.0);\n");
        }
        /* the code runs after the declarations above: head holds both, and the two pieces were built in that order */
        if (m->body.p) d3sb_printf(&head, "%s", m->body.p);
        if (!s->ps) {
            d3sb_printf(&head, "    G2_POINT(v_psz.x);\n");
            if (m->out_fog) d3sb_printf(&head, "    v_fog = v_fogv.x;\n");
        } else {
            if (m->major == 1) d3sb_printf(&head, "    g2_out = r0;\n");
            if (m->depth_out) d3sb_printf(&head, "    gl_FragDepth = G2_FD(g2_depth.x);\n");      /* d3d9_ff.c: the depth as the target keeps it */
        }
        free(code.p);
    }
    if (m->err[0]) { snprintf(s->error, sizeof s->error, "%s", m->err); rc = -1; }
    s->body = head.p ? head.p : calloc(1, 1);
    s->decl = m->decl.p ? m->decl.p : calloc(1, 1);
    free(m->body.p);
    free(m);
    return rc;
}

/* ---------------------------------------------------------------- the same bytecode, run on the PC: d3d9_cpu.c */
#include <math.h>

typedef struct vm {
    sm c;                                       /* what read_ins reads */
    const d3shader *s;
    const d3state *st;
    const float (*v)[4];
    d3vsout *o;
    float r[32][4], p[4];
    int a[4], aL, defi[16][4], has_i[16], defb[16], has_b[16];
    uint8_t out_usage[12], out_index[12];
    const DWORD *label[32];
} vm;

static void vm_const(const vm *m, int r, float *o)
{
    const d3shader *s = m->s;
    if (r < 0 || r >= VS_CONSTS) { memset(o, 0, 16); return; }
    memcpy(o, s->def_mask[r / 32] >> (r % 32) & 1 ? s->defc + 4 * r : m->st->vc[r], 16);
}

static void vm_src(const vm *m, DWORD p, int rel, float *o)
{
    int ty = reg_type(p), r = reg_num(p), mod = (int)(p >> 24 & 15), i;
    DWORD sw = p >> 16 & 255;
    float t[4] = { 0, 0, 0, 0 }, x[4];
    if (rel == REL_LOOP) r += m->aL;
    else if (rel) r += m->a[rel - 1];
    switch (ty) {
    case R_TEMP: memcpy(t, m->r[r & 31], 16); break;
    case R_INPUT: memcpy(t, m->v[r & 31], 16); break;
    case R_CONST: vm_const(m, r, t); break;
    case R_CONSTINT: { const int *q = m->has_i[r & 15] ? m->defi[r & 15] : m->st->vi[r & 15]; for (i = 0; i < 4; i++) t[i] = (float)q[i]; break; }
    case R_CONSTBOOL: t[0] = t[1] = t[2] = t[3] = (m->has_b[r & 15] ? m->defb[r & 15] : m->st->vb_c[r & 15]) ? 1.0f : 0.0f; break;
    case R_LOOP: t[0] = t[1] = t[2] = t[3] = (float)m->aL; break;
    case R_PREDICATE: memcpy(t, m->p, 16); break;
    case R_ADDR: for (i = 0; i < 4; i++) t[i] = (float)m->a[i]; break;
    default: break;
    }
    for (i = 0; i < 4; i++) x[i] = t[sw >> (2 * i) & 3];
    for (i = 0; i < 4; i++) {
        float v = x[i];
        switch (mod) {
        case 1: v = -v; break;
        case 2: v = v - 0.5f; break;
        case 3: v = 0.5f - v; break;
        case 4: v = 2.0f * (v - 0.5f); break;
        case 5: v = -2.0f * (v - 0.5f); break;
        case 6: v = 1.0f - v; break;
        case 7: v = 2.0f * v; break;
        case 8: v = -2.0f * v; break;
        case 9: v = v / x[2]; break;
        case 10: v = v / x[3]; break;
        case 11: v = fabsf(v); break;
        case 12: v = -fabsf(v); break;
        case 13: v = 1.0f - v; break;
        default: break;
        }
        o[i] = v;
    }
}

static void vm_dst(vm *m, const ins *x, const float *val)
{
    DWORD wm = x->d >> 16 & 15;
    int ty = reg_type(x->d), r = reg_num(x->d), mod = (int)(x->d >> 20 & 15), sh = (int)(x->d >> 24 & 15), k;
    float scale = sh ? (sh < 8 ? (float)(1 << sh) : 1.0f / (float)(1 << (16 - sh))) : 1.0f, pr[4] = { 1, 1, 1, 1 }, *o = NULL;
    d3vsout *q = m->o;
    if (x->tok & INS_PREDICATED) vm_src(m, x->pred, 0, pr);
    switch (ty) {
    case R_TEMP: o = m->r[r & 31]; break;
    case R_PREDICATE: o = m->p; break;
    case R_RASTOUT: if (r == 0) o = q->pos; else if (r == 1) { o = q->fog; q->wrote_fog = 1; } else { o = q->psize; q->wrote_psize = 1; } break;
    case R_ATTROUT: o = q->col[r & 1]; break;
    case R_OUTPUT:
        if (m->c.major >= 3 && r < 12) {
            int u = m->out_usage[r], i = m->out_index[r] & 11;
            if (u == D3DDECLUSAGE_POSITION) o = q->pos;
            else if (u == D3DDECLUSAGE_COLOR) o = q->col[i & 1];
            else if (u == D3DDECLUSAGE_TEXCOORD) o = q->tex[i];
            else if (u == D3DDECLUSAGE_FOG) { o = q->fog; q->wrote_fog = 1; }
            else if (u == D3DDECLUSAGE_PSIZE) { o = q->psize; q->wrote_psize = 1; }
        } else if (r < 12) o = q->tex[r];
        break;
    default: break;
    }
    if (!o) return;
    for (k = 0; k < 4; k++) {
        float v = val[k] * scale;
        if (!(wm >> k & 1) || pr[k] == 0.0f) continue;
        if (mod & 1) v = v < 0.0f ? 0.0f : v > 1.0f ? 1.0f : v;
        o[k] = v;
    }
}

static int cmp_f(int c, float a, float b)
{
    switch (c) { case 1: return a > b; case 2: return a == b; case 3: return a >= b; case 4: return a < b; case 5: return a != b; default: return a <= b; }
}

static void vm_ins(vm *m, const ins *x)
{
    float a[4] = { 0, 0, 0, 0 }, b[4] = { 0, 0, 0, 0 }, c[4] = { 0, 0, 0, 0 }, r[4] = { 0, 0, 0, 0 };
    int op = x->op, k;
    if (x->ns > 0) vm_src(m, x->s[0], x->srel[0], a);
    if (x->ns > 1 && !(op >= O_M4x4 && op <= O_M3x2)) vm_src(m, x->s[1], x->srel[1], b);
    if (x->ns > 2) vm_src(m, x->s[2], x->srel[2], c);
    if (!m->c.ps && (op == O_MOV || op == O_MOVA) && reg_type(x->d) == R_ADDR) {       /* vs 1.x floors, mova rounds */
        for (k = 0; k < 4; k++) if (x->d >> (16 + k) & 1) m->a[k] = (int)floorf(a[k] + (op == O_MOVA ? 0.5f : 0.0f));
        return;
    }
    if (op_is_scalar_src(op)) a[0] = a[1] = a[2] = a[3];
#define EACH(e) for (k = 0; k < 4; k++) r[k] = (e)
    switch (op) {
    case O_NOP: return;
    case O_MOV: case O_MOVA: memcpy(r, a, 16); break;
    case O_ADD: EACH(a[k] + b[k]); break;
    case O_SUB: EACH(a[k] - b[k]); break;
    case O_MUL: EACH(a[k] * b[k]); break;
    case O_MAD: EACH(a[k] * b[k] + c[k]); break;
    case O_LRP: EACH(a[k] * b[k] + (1.0f - a[k]) * c[k]); break;
    case O_RCP: EACH(1.0f / a[3]); break;
    case O_RSQ: EACH(1.0f / sqrtf(fabsf(a[3]))); break;
    case O_EXP: EACH(powf(2.0f, a[3])); break;
    case O_LOG: EACH(log2f(fabsf(a[3]))); break;
    case O_POW: EACH(powf(fabsf(a[3]), b[3])); break;
    case O_DP3: EACH(a[0] * b[0] + a[1] * b[1] + a[2] * b[2]); break;
    case O_DP4: EACH(a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3]); break;
    case O_MIN: EACH(a[k] < b[k] ? a[k] : b[k]); break;
    case O_MAX: EACH(a[k] > b[k] ? a[k] : b[k]); break;
    case O_SLT: EACH(a[k] < b[k] ? 1.0f : 0.0f); break;
    case O_SGE: EACH(a[k] >= b[k] ? 1.0f : 0.0f); break;
    case O_FRC: EACH(a[k] - floorf(a[k])); break;
    case O_ABS: EACH(fabsf(a[k])); break;
    case O_SGN: EACH(a[k] > 0.0f ? 1.0f : a[k] < 0.0f ? -1.0f : 0.0f); break;
    case O_NRM: { float l = 1.0f / sqrtf(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]); EACH(a[k] * l); break; }
    case O_CRS: r[0] = a[1] * b[2] - a[2] * b[1]; r[1] = a[2] * b[0] - a[0] * b[2]; r[2] = a[0] * b[1] - a[1] * b[0]; break;
    case O_LIT: { float w = a[3] < -128.0f ? -128.0f : a[3] > 128.0f ? 128.0f : a[3]; r[0] = r[3] = 1.0f; r[1] = a[0] > 0.0f ? a[0] : 0.0f; r[2] = a[0] > 0.0f ? powf(a[1] > 0.0f ? a[1] : 0.0f, w) : 0.0f; break; }
    case O_DST: r[0] = 1.0f; r[1] = a[1] * b[1]; r[2] = a[2]; r[3] = b[3]; break;
    case O_EXPP: r[0] = powf(2.0f, floorf(a[3])); r[1] = a[3] - floorf(a[3]); r[2] = powf(2.0f, a[3]); r[3] = 1.0f; break;
    case O_LOGP: { float l = fabsf(a[3]), f = floorf(log2f(l)); r[0] = f; r[1] = l / powf(2.0f, f); r[2] = log2f(l); r[3] = 1.0f; break; }
    case O_SINCOS: r[0] = cosf(a[3]); r[1] = sinf(a[3]); break;
    case O_SETP: EACH(cmp_f((int)(x->tok >> 16 & 7), a[k], b[k]) ? 1.0f : 0.0f); break;
    case O_M4x4: case O_M4x3: case O_M3x4: case O_M3x3: case O_M3x2: {
        int rows = op == O_M4x4 || op == O_M3x4 ? 4 : op == O_M3x2 ? 2 : 3, four = op == O_M4x4 || op == O_M4x3;
        for (k = 0; k < rows; k++) {
            float t[4];
            vm_src(m, (x->s[1] & ~(DWORD)0x7FF) | (DWORD)(reg_num(x->s[1]) + k), x->srel[1], t);
            r[k] = a[0] * t[0] + a[1] * t[1] + a[2] * t[2] + (four ? a[3] * t[3] : 0.0f);
        }
        break;
    }
    default: snprintf(m->c.err, sizeof m->c.err, "opcode %d on the PC", op); return;
    }
#undef EACH
    vm_dst(m, x, r);
}

/* past what a block of code leaves out: at its else or end, whichever belongs to it */
static const DWORD *vm_skip(vm *m, const DWORD *p)
{
    int nest = 0;
    while (p < m->c.end && (*p & 0xFFFF) != O_END) {
        int op = (int)(*p & 0xFFFF);
        const DWORD *q = p;
        ins x;
        if (op == O_COMMENT) { p += 1 + (*p >> 16); continue; }
        if (op == O_DCL) { p += 3; continue; }
        if (op == O_DEF || op == O_DEFI) { p += 6; continue; }
        if (op == O_DEFB) { p += 3; continue; }
        memset(&x, 0, sizeof x);
        if (read_ins(&m->c, &q, &x) < 0) return m->c.end;
        if (op == O_IF || op == O_IFC || op == O_LOOP || op == O_REP) nest++;
        else if (op == O_ENDIF || op == O_ENDLOOP || op == O_ENDREP) { if (!nest) return p; nest--; }
        else if (op == O_ELSE && !nest) return p;
        p = q;
    }
    return p;
}

/* runs from p to the token that ends its block, which it returns. *brk: a break left it, the loop around it is over */
static const DWORD *vm_block(vm *m, const DWORD *p, int *brk, int depth)
{
    while (p < m->c.end && !m->c.err[0]) {
        DWORD t = *p;
        int op = (int)(t & 0xFFFF), i;
        const DWORD *q = p;
        ins x;
        float a[4], b[4];
        if (op == O_END) return p;
        if (op == O_COMMENT) { p += 1 + (t >> 16); continue; }
        if (op == O_DCL) { p += 3; continue; }
        if (op == O_DEF || op == O_DEFI) { p += 6; continue; }
        if (op == O_DEFB) { p += 3; continue; }
        memset(&x, 0, sizeof x);
        if (read_ins(&m->c, &q, &x) < 0) return p;
        switch (op) {
        case O_ELSE: case O_ENDIF: case O_ENDLOOP: case O_ENDREP: case O_RET: case O_LABEL: return p;
        case O_BREAK: *brk = 1; return p;
        case O_BREAKC: vm_src(m, x.s[0], x.srel[0], a); vm_src(m, x.s[1], x.srel[1], b); if (cmp_f((int)(t >> 16 & 7), a[0], b[0])) { *brk = 1; return p; } break;
        case O_BREAKP: vm_src(m, x.s[0], x.srel[0], a); if (a[0] != 0.0f) { *brk = 1; return p; } break;
        case O_IF: case O_IFC: {
            int c;
            const DWORD *e;
            vm_src(m, x.s[0], x.srel[0], a);
            if (op == O_IFC) { vm_src(m, x.s[1], x.srel[1], b); c = cmp_f((int)(t >> 16 & 7), a[0], b[0]); }
            else c = a[0] != 0.0f;
            e = c ? vm_block(m, q, brk, depth) : vm_skip(m, q);
            if (*brk) return e;
            if (e < m->c.end && (*e & 0xFFFF) == O_ELSE) {
                e = c ? vm_skip(m, e + 1) : vm_block(m, e + 1, brk, depth);
                if (*brk) return e;
            }
            p = e + 1;
            continue;
        }
        case O_REP: case O_LOOP: {
            int r = reg_num(x.s[op == O_LOOP ? 1 : 0]) & 15, save = m->aL, n, by, inner = 0;
            const int *iv = m->has_i[r] ? m->defi[r] : m->st->vi[r];
            const DWORD *e = NULL;
            n = iv[0]; by = iv[2];
            if (op == O_LOOP) m->aL = iv[1];
            for (i = 0; i < n && !m->c.err[0]; i++) {
                e = vm_block(m, q, &inner, depth);
                if (inner) break;
                if (op == O_LOOP) m->aL += by;
            }
            m->aL = save;
            p = vm_skip(m, q) + 1;
            (void)e;
            continue;
        }
        case O_CALL: case O_CALLNZ: {
            int l = reg_num(x.s[0]) & 31, c = 1, inner = 0;
            if (op == O_CALLNZ) { vm_src(m, x.s[1], x.srel[1], a); c = a[0] != 0.0f; }
            if (depth >= 8 || !m->label[l]) { snprintf(m->c.err, sizeof m->c.err, "a call that cannot be made"); return p; }
            if (c) vm_block(m, m->label[l], &inner, depth + 1);
            break;
        }
        default: vm_ins(m, &x); break;
        }
        p = q;
    }
    return p;
}

int d3_vs_cpu(const d3state *st, const d3shader *s, const float (*in)[4], d3vsout *o)
{
    vm *m = (vm *)calloc(1, sizeof *m);
    const DWORD *p;
    int brk = 0, rc;
    if (!m) return -1;
    memset(o, 0, sizeof *o);
    o->pos[3] = 1.0f;
    o->col[0][3] = o->col[1][3] = 1.0f;
    m->s = s; m->st = st; m->v = in; m->o = o;
    m->c.s = (d3shader *)s; m->c.end = s->tok + s->ntok; m->c.major = s->major; m->c.minor = s->minor; m->c.ps = s->ps;
    for (p = s->tok + 1; p < m->c.end && (*p & 0xFFFF) != O_END; ) {
        int op = (int)(*p & 0xFFFF);
        ins x;
        if (op == O_COMMENT) { p += 1 + (*p >> 16); continue; }
        if (op == O_DCL) {
            if (reg_type(p[2]) == R_OUTPUT && reg_num(p[2]) < 12) { m->out_usage[reg_num(p[2])] = (uint8_t)(p[1] & 31); m->out_index[reg_num(p[2])] = (uint8_t)(p[1] >> 16 & 15); }
            p += 3;
            continue;
        }
        if (op == O_DEF) { p += 6; continue; }
        if (op == O_DEFI) { memcpy(m->defi[reg_num(p[1]) & 15], p + 2, 16); m->has_i[reg_num(p[1]) & 15] = 1; p += 6; continue; }
        if (op == O_DEFB) { m->defb[reg_num(p[1]) & 15] = p[2] != 0; m->has_b[reg_num(p[1]) & 15] = 1; p += 3; continue; }
        memset(&x, 0, sizeof x);
        if (read_ins(&m->c, &p, &x) < 0) break;
        if (x.op == O_LABEL) m->label[reg_num(x.s[0]) & 31] = p;
    }
    vm_block(m, s->tok + 1, &brk, 0);
    rc = m->c.err[0] ? -1 : 0;
    free(m);
    return rc;
}
