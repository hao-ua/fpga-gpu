#!/usr/bin/env python3
"""isa64 — the 64-bit swizzling shader ISA:
encoder / decoder / assembler / static checker, the scalar fragment reference simulator (one warp = 4 quads = 16
lanes, one micro-op per written component) and the vertex flavour of the same encoding on the same warp model
(16 vertices per warp, LDV = attributes, STV / STO 0 / STO 1 outputs, no BRP, no fragment-only ops).

Word (64 bits):
  op[63:57] wm[56:53] rd[52:48] rs0[47:43] swz0[42:35] m0[34:33] rs1[32:28] swz1[27:20] m1[19:18]
  rs2[17:13] swz2[12:5] m2[4:3] x[2:0]
  swz = 4 x 2-bit component selects, bits [2c+1:2c] = the source component read for destination component c
  (identity 0xE4); m = source modifier 0 none / 1 neg / 2 abs / 3 -abs.
  Immediates overlay the low bits: BRA IF ELSE LOOP imm16 [15:0]; LDP LDV STV index [7:0] (+ swz0 for LDP / LDV);
  TEX sampler [3:0] mode [5:4] coordinate set [9:6] (the components the core reads into the TMU request, one
  micro-op each: 0 = all four, the legacy encoding); SET SETP cc [16:13]; IOP fn [16:13];
  MOVI fp24 [27:4]; BRP branch-if-false = bit 52.

Assembler syntax (a superset of the 32-bit one, so every tests/*.sasm assembles unchanged):
  OP[.wm] rd[.wm], src, src, src      src = [-][!]rN[.swz]     .swz: 1 letter = splat, 4 = full, 2-3 = the last repeats
  LDP rd, idx[.swz]    LDV rd, idx[.swz]    MOVI rd, #1.5 | #0x3f0000    TEX rd, rs0[.swz], #samp[, BIAS|LOD][, .xy]
  (TEX's trailing .set = the coordinate components the core fetches: .xy for a 2D sampler, .xyw with BIAS / LOD, .xyz
   for a cube map, none = all four)
  STO [outidx,] rs0[.swz]    STOZ rs0[.swz]    SETP cc, rs0, rs1    SET cc, rd, rs0, rs1    SEL rd, rc, ra, rb
  IF/ELSE/BRA/BRP label [F]    LOOP n    ENDL ENDIF BREAK CONT DISCARD END NOP    IAND.. (IOP mnemonics)

CLI (run from the repository root):
  python3 tools/isa64.py asm  tests/x.sasm [-o out.mem]       64-bit words, one per line (16 hex digits)
  python3 tools/isa64.py dis  file.mem
  python3 tools/isa64.py fs2  tests/x.sasm [--exact-rcp] [--no-interp] [--active HEX] [--deep]
                                                                golden set tools/gen/<x>_fs2_*.mem for tb_fs2 (FP24)
  python3 tools/isa64.py vs2  tests/x.sasm [--exact-rcp] [--active HEX] [--deep]
                                                                golden set tools/gen/<x>_vs2_*.mem for tb_vs2 (16 vertices)
  python3 tools/isa64.py selftest                               every tests/*.sasm: new encoder + simulators against
                                                                the 32-bit shader_ref ones (bit-exact), + unit checks
"""
import os, sys, re
from fractions import Fraction as F
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import float_ref as fr
import shader_ref as sr

GEN = sr.GEN
FMT = fr.FP24
NREG, NPARAM, NVARY, NWARP_LANES = 32, 64, 16, 16
IF_DEPTH, LOOP_DEPTH = 8, 4

OPS = dict(sr.OPS); OPS['MOVI'] = 2            # op 2 was reserved for an immediate move in the 32-bit ISA; now used
OPS['LDPX'] = 57                               # vertex core only: LDP indexed per lane by a register
# Fragment core only, board B builds that have them (shader_core_fs2 XSFU / XCLAMP). Each is ONE micro-op per component and, bit for bit, the sequence it replaces:
OPS['SQRT'] = 58                               # rd = x * RSQRT(x), and x itself where x is zero     = RSQRT t, x; MUL u, x, t; SEL rd, x, u, x
OPS['RCPM'] = 59                               # rd = y * RCP(x)    (assembler: DIV rd, y, x)         = RCP t, x; MUL rd, y, t
OPS['RSQM'] = 60                               # rd = y * RSQRT(x)                                    = RSQRT t, x; MUL rd, y, t
OPS['CLAMP'] = 61                              # rd = MIN(MAX(x, lo), hi)                             = MAX t, x, lo; MIN rd, t, hi
XOPS = ('SQRT', 'RCPM', 'RSQM', 'CLAMP')
RND_OPS, IOP_FN, CMP, INT_OPS = sr.RND_OPS, sr.IOP_FN, sr.CMP, sr.INT_OPS
OPNAME = {v: k for k, v in OPS.items() if k not in ('STOZ', 'BRP')}   # 21 printed as BRP (VS) / STOZ (FS) by context
SWZ_ID = 0xE4
ONE = fr.encode_real(F(1), FMT); ZERO = 0

# ---- operand classes (what the fields mean per opcode) ----
ARITH2 = ('ADD', 'SUB', 'MUL', 'MIN', 'MAX', 'DP3', 'DP4', 'RCPM', 'RSQM')   # rd, rs0, rs1 (+ modifiers)
ARITH1 = ('MOV', 'RCP', 'RSQRT', 'LOG2', 'EXP2', 'SIN', 'COS', 'FTOI', 'ITOF', 'FTOU', 'UTOF', 'DDX', 'DDY', 'SQRT') + RND_OPS
ARITH3 = ('MAD', 'CLAMP')                                             # rd, rs0, rs1, rs2 (+ modifiers)
INT2 = ('IADD', 'ISUB', 'ISLT', 'ISGE', 'ISEQ', 'ISNE', 'IMUL', 'IDIV', 'IMOD', 'UDIV', 'UMOD')
CTRL_IMM = ('BRA', 'BRP', 'LOOP', 'IF', 'ELSE')
CTRL0 = ('NOP', 'ENDL', 'ENDIF', 'BREAK', 'CONT', 'DISCARD', 'END')
FS_ONLY = ('TEX', 'DDX', 'DDY', 'DISCARD', 'STOZ') + XOPS           # fragment core only: rejected in a vertex program
VS_ONLY = ('LDPX',)                                                  # vertex core only: ES2 indexes uniforms at run time in the VS alone
WRITES = set(ARITH2 + ARITH1 + INT2 + ARITH3 + ('SEL', 'SET', 'LDP', 'LDV', 'LDPX', 'TEX', 'MOVI')) | set(IOP_FN)


class AsmError(ValueError): pass


# ================================================================ encoding
def enc_swz(s):
    """'xyzw'-string -> 8-bit field (1 letter splat, 4 letters full, 2-3 letters: the last repeats)."""
    s = (s or 'xyzw').lower()
    if len(s) > 4 or any(ch not in 'xyzw' for ch in s): raise AsmError("bad swizzle ." + s)
    if len(s) == 1: s = s * 4
    while len(s) < 4: s += s[-1]
    v = 0
    for c in range(4): v |= 'xyzw'.index(s[c]) << (2 * c)
    return v


def swz_str(v):
    s = ''.join('xyzw'[(v >> (2 * c)) & 3] for c in range(4))
    return s[0] if s == s[0] * 4 else s                          # a splat prints as one letter


def swz_comp(v, c): return (v >> (2 * c)) & 3


def encode(op, wm=0xF, rd=0, s0=(0, SWZ_ID, 0), s1=(0, SWZ_ID, 0), s2=(0, SWZ_ID, 0), imm=0, immw=0):
    """Fields -> 64-bit word. imm / immw: an immediate overlaid on the low immw bits (immw 16, 24 at [27:4], ...)."""
    w = ((op & 0x7F) << 57) | ((wm & 0xF) << 53) | ((rd & 0x1F) << 48)
    w |= ((s0[0] & 0x1F) << 43) | ((s0[1] & 0xFF) << 35) | ((s0[2] & 3) << 33)
    w |= ((s1[0] & 0x1F) << 28) | ((s1[1] & 0xFF) << 20) | ((s1[2] & 3) << 18)
    w |= ((s2[0] & 0x1F) << 13) | ((s2[1] & 0xFF) << 5) | ((s2[2] & 3) << 3)
    if immw: w = (w & ~((1 << immw) - 1)) | (imm & ((1 << immw) - 1))
    return w


def decode(w):
    d = dict(op=(w >> 57) & 0x7F, wm=(w >> 53) & 0xF, rd=(w >> 48) & 0x1F,
             rs0=(w >> 43) & 0x1F, swz0=(w >> 35) & 0xFF, m0=(w >> 33) & 3,
             rs1=(w >> 28) & 0x1F, swz1=(w >> 20) & 0xFF, m1=(w >> 18) & 3,
             rs2=(w >> 13) & 0x1F, swz2=(w >> 5) & 0xFF, m2=(w >> 3) & 3,
             imm16=w & 0xFFFF, idx=w & 0xFF, samp=w & 0xF, mode=(w >> 4) & 3, cm=(w >> 6) & 0xF, cc=(w >> 13) & 0xF,
             fn=(w >> 13) & 0xF, imm24=(w >> 4) & 0xFFFFFF, brpf=(w >> 52) & 1)
    return d


# ================================================================ assembler
def _reg(t):
    m = re.fullmatch(r'[rR](\d+)', t)
    if not m or not (0 <= int(m.group(1)) < NREG): raise AsmError("bad register " + t)
    return int(m.group(1))


def parse_src(tok):
    """'[-][!]rN[.swz]' -> (reg, swz8, mod)."""
    t = tok; mod = 0
    while t and t[0] in '-!':
        if t[0] == '-': mod ^= 1
        else: mod |= 2
        t = t[1:]
    r, _, s = t.partition('.')
    return _reg(r), enc_swz(s), mod & 3


def parse_dst(tok):
    r, _, s = tok.partition('.')
    return _reg(r), (sr.maskbits(s) if s else None)


def parse_idx(tok):
    """'idx[.swz]' or '#idx[.swz]' -> (int, swz8)."""
    t = tok.lstrip('#'); n, _, s = t.partition('.')
    return int(n, 0), enc_swz(s)


def parse_fp(tok):
    t = tok.lstrip('#')
    if t.lower().startswith('0x'): return int(t, 16) & 0xFFFFFF
    return fr.encode_real(F(t), FMT)


def assemble(lines, vs=False, max_if=IF_DEPTH, max_loop=LOOP_DEPTH):
    """-> (words, labels). vs: op 21 is BRP (else STOZ). Raises AsmError (also for the static rule of check_overlap and the
    stack depths; max_if / max_loop = None assembles a deliberately overflowing program, e.g. tests/test_fsstk)."""
    labels = {}; pc = 0; prog = []
    for ln in lines:
        ln = ln.split('//')[0].split(';')[0].strip()
        if not ln: continue
        if ln.endswith(':'): labels[ln[:-1]] = pc; continue
        prog.append(ln); pc += 1
    words = []; ifd = 0; lpd = 0
    for ln in prog:
        t = re.split(r'[\s,]+', ln.strip())
        mn_full = t[0].upper(); mn, _, msuf = mn_full.partition('.')
        wm_mn = sr.maskbits(msuf) if msuf else None
        if mn == 'DIV':                              # DIV rd, y, x = y / x: RCPM with its operands in the core's order
            if len(t) < 4: raise AsmError("DIV rd, y, x: " + ln)
            mn = 'RCPM'; t = [t[0], t[1], t[3], t[2]] + t[4:]
        if mn in IOP_FN: op = OPS['IOP']
        elif vs and mn in FS_ONLY: raise AsmError(f"{mn} in a vertex program")
        elif vs and mn == 'BRP': raise AsmError("BRP in a vertex program: the 16-lane vertex core has no scalar branch, use IF / ELSE")
        elif not vs and mn in ('BRP', 'STV'): raise AsmError(f"{mn} in a fragment program")
        elif mn in OPS: op = OPS[mn]
        else: raise AsmError("bad op " + mn)
        args = [a for a in t[1:] if a != '']

        def dst(i):
            r, wm = parse_dst(args[i]); wm = wm if wm is not None else (wm_mn if wm_mn is not None else 0xF)
            if wm == 0: raise AsmError("empty write mask in: " + ln)
            return r, wm
        if mn in ARITH2 or mn in INT2:
            rd, wm = dst(0); s0 = parse_src(args[1]); s1 = parse_src(args[2])
            if mn in INT2 and (s0[2] or s1[2]): raise AsmError("no modifiers on integer ops: " + ln)
            w = encode(op, wm, rd, s0, s1)
        elif mn in IOP_FN:
            rd, wm = dst(0); s0 = parse_src(args[1]); s1 = parse_src(args[2]) if len(args) > 2 else (0, SWZ_ID, 0)
            w = encode(op, wm, rd, s0, s1) | (IOP_FN[mn] << 13)
        elif mn in ARITH3:
            rd, wm = dst(0); w = encode(op, wm, rd, parse_src(args[1]), parse_src(args[2]), parse_src(args[3]))
        elif mn in ARITH1:
            rd, wm = dst(0); w = encode(op, wm, rd, parse_src(args[1]))
        elif mn == 'SEL':                            # no modifiers (an exact select; the core reads the registers raw)
            rd, wm = dst(0); ss = [parse_src(args[k]) for k in (1, 2, 3)]
            if any(s[2] for s in ss): raise AsmError("no modifiers on SEL: " + ln)
            w = encode(op, wm, rd, ss[0], ss[1], ss[2])
        elif mn == 'MOVI':
            rd, wm = dst(0); w = encode(op, wm, rd, s1=(0, 0, 0), s2=(0, 0, 0)) | (parse_fp(args[1]) << 4)
        elif mn == 'TEX':
            rd, wm = dst(0); mode = 0; cm = 0
            for a in args[3:]:
                if a.upper() in ('BIAS', 'LOD'): mode = {'BIAS': 1, 'LOD': 2}[a.upper()]
                elif a.startswith('.'): cm = sr.maskbits(a[1:])
                else: raise AsmError("bad TEX option in: " + ln)
            samp = int(args[2].lstrip('#'), 0)
            if not 0 <= samp < 16: raise AsmError("bad sampler in: " + ln)
            w = encode(op, wm, rd, parse_src(args[1]), imm=(cm << 6) | (mode << 4) | samp, immw=10)
        elif mn == 'STOZ':
            w = encode(op, 0, 0, parse_src(args[0]))
        elif mn == 'SETP':
            w = encode(op, 0, 0, parse_src(args[1]), parse_src(args[2])) | (CMP[args[0].upper()] << 13)
        elif mn == 'SET':
            rd, wm = dst(1); w = encode(op, wm, rd, parse_src(args[2]), parse_src(args[3])) | (CMP[args[0].upper()] << 13)
        elif mn in ('LDP', 'LDV'):
            rd, wm = dst(0); idx, swz = parse_idx(args[1])
            if (mn == 'LDP' and not 0 <= idx < (256 if vs else NPARAM)) or (mn == 'LDV' and not 0 <= idx < NVARY):
                raise AsmError("index out of range in: " + ln)
            w = encode(op, wm, rd, (0, swz, 0), imm=idx, immw=8)
        elif mn == 'LDPX':
            # LDPX rd[.wm], idx[.swz], rN[.swz]: rd.c = params[(idx + rN[swz1[c]]) mod 256][swz0[c]] per lane (the
            # register holds an int; its low 8 bits are added). Vertex programs only; no modifier on the index.
            if not vs: raise AsmError("LDPX in a fragment program: " + ln)
            rd, wm = dst(0); idx, swz = parse_idx(args[1]); ix = parse_src(args[2])
            if not 0 <= idx < 256: raise AsmError("index out of range in: " + ln)
            if ix[2]: raise AsmError("no modifier on LDPX's index register: " + ln)
            w = encode(op, wm, rd, (0, swz, 0), ix, imm=idx, immw=8)
        elif mn in ('STV', 'STO'):
            if len(args) == 2: outidx, src = int(args[0], 0), args[1]
            else: outidx, src = 0, args[0]
            # STV 0..15; STO 0 = gl_Position / gl_FragColor, STO 1 = gl_PointSize (vertex programs only)
            if (mn == 'STV' and not 0 <= outidx < NVARY) or (mn == 'STO' and not 0 <= outidx <= (1 if vs else 0)):
                raise AsmError("output index out of range in: " + ln)
            w = encode(op, 0, 0, parse_src(src), imm=outidx, immw=8)
        elif mn in CTRL_IMM:
            tgt = labels[args[0]] if args[0] in labels else int(args[0], 0)
            w = encode(op, 0, 0, imm=tgt, immw=16)
            if mn == 'BRP' and len(args) > 1 and args[1].upper() == 'F': w |= 1 << 52
            if mn == 'IF': ifd += 1
            elif mn == 'LOOP': lpd += 1
            if max_if is not None and ifd > max_if: raise AsmError(f"IF nesting deeper than {max_if}")
            if max_loop is not None and lpd > max_loop: raise AsmError(f"LOOP nesting deeper than {max_loop}")
        elif mn in CTRL0:
            w = encode(op, 0, 0)
            if mn == 'ENDIF': ifd -= 1
            elif mn == 'ENDL': lpd -= 1
        else:
            raise AsmError("bad op " + mn)
        check_overlap(w, ln)
        words.append(w)
    return words, labels


def check_overlap(w, ln=''):
    """The static rule: a source may not read a component of rd that an earlier micro-op of the same instruction wrote
    (micro-ops run x, y, z, w in order). Chains (DP3 / DP4, LDV) and TEX read everything before any write."""
    d = decode(w); op = d['op']; name = OPNAME.get(op, '?')
    if name not in WRITES or name in ('DP3', 'DP4', 'LDV', 'LDP', 'TEX', 'MOVI'): return
    nsrc = 3 if name in ARITH3 + ('SEL',) else 1 if name in ARITH1 else 2
    srcs = [(d['rs0'], d['swz0']), (d['rs1'], d['swz1']), (d['rs2'], d['swz2'])][:nsrc]
    if name == 'IOP' and d['fn'] in (3, 11, 12, 13): srcs = srcs[:1]          # INOT IABS INEG ISIGN read rs0 only
    if name == 'LDPX': srcs = [(d['rs1'], d['swz1'])]                           # the index register (rs1) per micro-op
    written = []
    for c in range(4):
        if not (d['wm'] >> c) & 1: continue
        for r, swz in srcs:
            if r == d['rd'] and swz_comp(swz, c) in written:
                raise AsmError(f"dest/source overlap (component {'xyzw'[swz_comp(swz, c)]} written before it is "
                               f"read for .{'xyzw'[c]}): {ln}")
        written.append(c)


def disasm(w, vs=False):
    d = decode(w); op = d['op']; name = OPNAME.get(op, f'OP{op}')
    if op == 21: name = 'STOZ' if not vs else 'OP21'                 # op 21 is not an instruction of the vertex core
    if op == OPS['IOP']: name = {v: k for k, v in IOP_FN.items()}[d['fn']]
    wm = ''.join('xyzw'[c] for c in range(4) if (d['wm'] >> c) & 1)

    def src(k):
        r, s, m = d[f'rs{k}'], d[f'swz{k}'], d[f'm{k}']
        return ('-' if m & 1 else '') + ('!' if m & 2 else '') + f"r{r}" + ('' if s == SWZ_ID else '.' + swz_str(s))
    rd = f"r{d['rd']}" + ('' if d['wm'] == 0xF else '.' + wm)
    if name in ARITH2 or name in INT2 or name in IOP_FN: return f"{name} {rd}, {src(0)}, {src(1)}"
    if name in ARITH1: return f"{name} {rd}, {src(0)}"
    if name in ARITH3 + ('SEL',): return f"{name} {rd}, {src(0)}, {src(1)}, {src(2)}"
    if name == 'MOVI': return f"MOVI {rd}, #0x{d['imm24']:06x}"
    if name == 'TEX': return (f"TEX {rd}, {src(0)}, #{d['samp']}" + ('', ', BIAS', ', LOD', ', ?')[d['mode']]
                              + ('' if d['cm'] in (0, 0xF) else ', .' + ''.join('xyzw'[c] for c in range(4) if (d['cm'] >> c) & 1)))
    if name in ('LDP', 'LDV'): return f"{name} {rd}, {d['idx']}" + ('' if d['swz0'] == SWZ_ID else '.' + swz_str(d['swz0']))
    if name == 'LDPX': return f"LDPX {rd}, {d['idx']}" + ('' if d['swz0'] == SWZ_ID else '.' + swz_str(d['swz0'])) + f", {src(1)}"
    if name in ('STO', 'STV'): return f"{name} {d['idx']}, {src(0)}"
    if name == 'STOZ': return f"STOZ {src(0)}"
    if name in ('SET',): return f"SET {list(CMP)[list(CMP.values()).index(d['cc'])]}, {rd}, {src(0)}, {src(1)}"
    if name in ('SETP',): return f"SETP {list(CMP)[list(CMP.values()).index(d['cc'])]}, {src(0)}, {src(1)}"
    if name in CTRL_IMM: return f"{name} {d['imm16']}" + (' F' if name == 'BRP' and d['brpf'] else '')
    return name


# ================================================================ scalar evaluation (shared by both simulators)
def _mod(x, m):
    if m == 0: return x
    sb = 1 << (FMT.W - 1)
    return (x ^ sb) if m == 1 else (x & ~sb) if m == 2 else (x | sb)


def _madd(a, b, c, sub=0):
    return fr.fadd(fr.fmul(a, b, FMT), (c ^ (1 << (FMT.W - 1))) if sub else c, FMT)


class Units:
    """The unit models (approximate SFU as shader_ref; exact_rcp = the RTL's truncated-Newton RCP of b_ref.py)."""
    def __init__(self, exact_rcp=False):
        self.sk = fr.SIN_KSIN[FMT.name]; self.sc = fr.SIN_CFRAC[FMT.name]; self.sC = fr.sin_C(FMT)
        self.slut = fr.gen_sin_lut(FMT, self.sk)
        self.exact_rcp = exact_rcp
        if exact_rcp:
            import b_ref
            self.rcp = b_ref.rcp
    def sfu(self, name, x):
        v = sr._sr(x, FMT)
        if name == 'RCP':
            if self.exact_rcp: return self.rcp(x)
            return sr._enc_sfu(sr._frcp(v), FMT)
        if name == 'RSQRT': return sr._enc_sfu(sr._frsqrt(v), FMT)
        if name == 'LOG2': return sr._enc_sfu(sr._flog2(v), FMT)
        if name == 'EXP2': return sr._enc_sfu(sr._fexp2(v), FMT)
        if name == 'SIN': return fr.sfu_sin(x, FMT, self.slut, self.sk, self.sc, self.sC, False)
        return fr.sfu_sin(x, FMT, self.slut, self.sk, self.sc, self.sC, True)


def eval_scalar(name, a, b, c, d, u):
    """One lane, one component: the result bits of a component-wise op on (modified) operand bits a, b, c."""
    if name == 'MOV': return a
    if name == 'ADD': return _madd(a, ONE, b)
    if name == 'SUB': return _madd(a, ONE, b, 1)
    if name == 'MUL': return _madd(a, b, ZERO)
    if name == 'MAD': return _madd(a, b, c)
    if name == 'MIN': return a if sr._sr(a, FMT) < sr._sr(b, FMT) else b
    if name == 'MAX': return a if sr._sr(a, FMT) > sr._sr(b, FMT) else b
    if name == 'SET': return ONE if sr._setp(a, b, d['cc'], FMT) else 0
    if name in ('RCP', 'RSQRT', 'LOG2', 'EXP2', 'SIN', 'COS'): return u.sfu(name, a)
    # the unit's result times an operand: the MUL micro-op's product (a zero product is +0: float_madd adds +0)
    if name == 'SQRT':                               # SEL rd, x, x * RSQRT(x), x: a zero x (FTZ: exponent 0) stays itself
        return _madd(a, u.sfu('RSQRT', a), ZERO) if (a >> FMT.M) & ((1 << FMT.E) - 1) else a
    if name == 'RCPM': return _madd(b, u.sfu('RCP', a), ZERO)
    if name == 'RSQM': return _madd(b, u.sfu('RSQRT', a), ZERO)
    if name == 'CLAMP':                              # MIN(MAX(x, lo), hi) by this function's MIN and MAX
        t = a if sr._sr(a, FMT) > sr._sr(b, FMT) else b
        return t if sr._sr(t, FMT) < sr._sr(c, FMT) else c
    if name == 'FTOI': return sr.ftoi(a, FMT)
    if name == 'ITOF': return sr.itof(a, FMT)
    if name == 'FTOU': return sr.ftou(a, FMT)
    if name == 'UTOF': return sr.utof(a, FMT)
    if name in RND_OPS: return fr.fround(a, FMT, OPS[name] - OPS['TRUNC'])
    if name == 'SEL': return sr.sel_lane(a, b, c, FMT)
    if name in INT2: return sr.int_op([a], [b], sr._intcode(OPS[name]), FMT, 1)[0]
    if name == 'IOP': return sr.int_op([a], [b], 16 + d['fn'], FMT, 1)[0]
    raise ValueError("eval_scalar: " + name)


def dot_tree(av, bv, n):
    """DP3 / DP4 of already-swizzled operand lists (p_k = a_k * b_k): (p0 + p1) + (p2 + p3), DP3 with p3 = +0 —
    bit-identical to float_dot4_sum and to the core's two-accumulator chain (step k: MAD(a_k, b_k, acc), acc = +0 at
    the start of each half; a zero sum is +0, so fadd(p, +0) = p)."""
    p = [fr.fmul(av[k], bv[k], FMT) for k in range(n)]
    s01 = fr.fadd(p[0], p[1], FMT)
    s23 = fr.fadd(p[2], p[3], FMT) if n == 4 else fr.fadd(p[2], ZERO, FMT)
    return fr.fadd(s01, s23, FMT)


# ================================================================ fragment simulator (one warp)
class FSInputs:
    """Per-warp inputs. lanes = 16 (lane l: quad l >> 2, pixel l & 3 = TL TR BL BR).
    params[i]    vec4 bits (64 entries)
    varin[l][i]  built-in rows per lane, i in 8..10 (gl_FragCoord, gl_FrontFacing, gl_PointCoord) as vec4 bits; with
                 interp=True (the board-B core) LDV 9 reads (row 9 .x of the quad's first pixel != 0 ? 1.0 : 0, 0, 0, 0),
                 LDV 10 (row 10 .x, .y, 0, 1.0), LDV 11..15 zeros; with interp=False every LDV i (0..15) reads
                 varin[l][i] (the single-board semantics, core INTERP = 0)
    lam[l]       (l1, l2) per lane (interp=True);  tslot[q] triangle slot of quad q;  tstore[slot][i] = (f0, D1, D2)
                 vec4s of user varying i (interp=True)
    active       initial active mask (partial warps);  tex_fn(sampler, mode, coords16) per quad as shader_ref
                 (tex_model = the bench's texture unit; None = TEX writes nothing, the 32-bit selftest twin)"""
    def __init__(self, params=None, varin=None, lam=None, tslot=None, tstore=None, active=0xFFFF, interp=True, tex_fn=None):
        self.params = params or [[0] * 4 for _ in range(NPARAM)]
        self.varin = varin or [[[0] * 4 for _ in range(NVARY)] for _ in range(16)]
        self.lam = lam or [(0, 0)] * 16
        self.tslot = tslot or [0] * 4
        self.tstore = tstore or [[([0] * 4, [0] * 4, [0] * 4) for _ in range(8)] for _ in range(8)]
        self.active = active; self.interp = interp; self.tex_fn = tex_fn


class FSResult:
    """outv[i][lane] (STV i) and psize[lane] (the STO 1 row) are the vertex flavour's outputs (simulate_vs2)."""
    def __init__(self, gpr, outc, killed, outz, err, steps, uops, outv=None, psize=None):
        self.gpr, self.outc, self.killed, self.outz, self.err, self.steps, self.uops = gpr, outc, killed, outz, err, steps, uops
        self.outv, self.psize = outv, psize


def tex_model(samp, mode, coords):
    """The bench's texture unit (tb_fs2 tmu_model2), bit-exact and per pixel so that a pixel / lane / component mix-up
    shows: coords[p*4 + k] = coordinate component k (s, t, r, q) of pixel p -> texels[p*4 + c]:
    R = s;  G = t's sign and exponent with q's low 16 mantissa bits (0 when t is zero);  B = 2 r (the exponent + 1
    for a normal r, else r);  A = float(4 samp + mode + 1). Every coordinate component, the sampler and the mode
    reach a texel."""
    out = [0] * 16
    a = fr.encode_real(F(4 * samp + mode + 1), FMT)
    for p in range(4):
        s, t, r, q = coords[p * 4: p * 4 + 4]
        te = (t >> FMT.M) & ((1 << FMT.E) - 1); re = (r >> FMT.M) & ((1 << FMT.E) - 1)
        out[p * 4 + 0] = s
        out[p * 4 + 1] = 0 if te == 0 else ((t & ~0xFFFF) | (q & 0xFFFF))
        out[p * 4 + 2] = r if re in (0, (1 << FMT.E) - 1) else r + (1 << FMT.M)
        out[p * 4 + 3] = a
    return out


def simulate_fs2(words, inp, units=None, max_steps=200000, vs=False):
    """Scalar warp simulator: gpr[r][lane][comp] bits; returns FSResult (outc[lane][comp], outz[lane] or None,
    err = a stack violation killed the warp). uops counts micro-op cycles the core would issue.
    vs: the vertex flavour (shader_core_vs2) — the lanes are vertices, LDV reads inp.varin (the
    attributes; inp.interp False), STV i writes outv[i], STO 0 the position (outc), STO 1 the point size (psize); the
    fragment-only ops and op 21 are errors; a stack violation stops the warp (err) without a kill."""
    u = units or Units()
    gpr = [[[0] * 4 for _ in range(16)] for _ in range(NREG)]
    outc = [[0] * 4 for _ in range(16)]; outz = [0] * 16; wrote_z = False
    outv = [[[0] * 4 for _ in range(16)] for _ in range(NVARY)]; psize = [[0] * 4 for _ in range(16)]
    active = inp.active & 0xFFFF; killed = 0; pred = 0; mstack = []; lstack = []
    pc = 0; steps = 0; uops = 0; err = False
    L = range(16)
    while steps < max_steps:
        steps += 1
        w = words[pc] if pc < len(words) else (OPS['END'] << 57)
        d = decode(w); op = d['op']; name = OPNAME.get(op, '?')
        if op == 21: name = 'STOZ'
        if op == OPS['IOP']: name = 'IOP'
        if vs and name in FS_ONLY: raise ValueError(f"simulate_vs2: {name} (op {op}) at {pc} is not a vertex-core op")
        if not vs and name in VS_ONLY: raise ValueError(f"simulate_fs2: {name} (op {op}) at {pc} is not a fragment-core op")
        wm = d['wm']; rd = d['rd']; nxt = pc + 1
        comps = [c for c in range(4) if (wm >> c) & 1]

        def src(k, c):          # operand k of destination component c, per lane (modified)
            r, swz, m = d[f'rs{k}'], d[f'swz{k}'], d[f'm{k}']
            cc = swz_comp(swz, c)
            return [_mod(gpr[r][l][cc], m) for l in L]

        def write(c, vals, mask=None):
            mk = active if mask is None else mask
            for l in L:
                if (mk >> l) & 1: gpr[rd][l][c] = vals[l]
        # ---------------- component-wise ops
        if name in ARITH2 + ARITH1 + INT2 + ARITH3 + ('SEL', 'SET', 'IOP') and name not in ('DP3', 'DP4'):
            for c in comps:
                a = src(0, c)
                b = src(1, c) if name not in ARITH1 else [0] * 16
                cc3 = src(2, c) if name in ARITH3 + ('SEL',) else [0] * 16
                if name in ('DDX', 'DDY'):
                    res = [0] * 16
                    for l in L:
                        q, p = l >> 2, l & 3
                        hi, lo = ((p | 1), (p & ~1)) if name == 'DDX' else ((p | 2), (p & ~2))
                        res[l] = _madd(a[4 * q + hi], ONE, a[4 * q + lo], 1)
                elif name in INT2 or name == 'IOP':
                    res = sr.int_op(a, b, sr._intcode(op, d['fn']), FMT, 16)
                else:
                    res = [eval_scalar(name, a[l], b[l], cc3[l], d, u) for l in L]
                write(c, res); uops += 1
        elif name in ('DP3', 'DP4'):
            n = 3 if name == 'DP3' else 4
            av = [src(0, k) for k in range(n)]; bv = [src(1, k) for k in range(n)]
            res = [dot_tree([av[k][l] for k in range(n)], [bv[k][l] for k in range(n)], n) for l in L]
            for c in comps: write(c, res)
            uops += (5 if n == 4 else 4) + len(comps) - 1
        elif name == 'MOVI':
            for c in comps: write(c, [d['imm24']] * 16); uops += 1
        elif name == 'LDP':
            for c in comps:
                write(c, [inp.params[d['idx']][swz_comp(d['swz0'], c)]] * 16); uops += 1
        elif name == 'LDPX':
            # per lane: slot = (idx + the low 8 bits of the index register's component swz1[c]) mod 256 — a 2's-complement
            # int in the lane, so a negative index wraps; the value's component is swz0[c] (as LDP)
            for c in comps:
                sc = swz_comp(d['swz0'], c); ic = swz_comp(d['swz1'], c)
                write(c, [inp.params[(d['idx'] + gpr[d['rs1']][l][ic]) & 0xFF][sc] for l in L]); uops += 1
        elif name == 'LDV':
            i = d['idx']
            for c in comps:
                sc = swz_comp(d['swz0'], c)
                if inp.interp and i < 8:
                    res = [0] * 16
                    for l in L:
                        f0, d1, d2 = inp.tstore[inp.tslot[l >> 2]][i]
                        l1, l2 = inp.lam[l]
                        res[l] = _madd(l2, d2[sc], _madd(l1, d1[sc], f0[sc]))
                    uops += 1          # one issued micro-op per component (the second step runs by itself)
                elif inp.interp:
                    # the board-B core's built-ins: gl_FragCoord per pixel; gl_FrontFacing one bit per
                    # quad (row 9 .x of the quad's first pixel != 0) read as (1.0 | 0, 0, 0, 0); gl_PointCoord (mu1, mu2,
                    # 0, 1.0); LDV 11..15 = 0
                    if i == 8: res = [inp.varin[l][8][sc] for l in L]
                    elif i == 9: res = [(ONE if (sc == 0 and inp.varin[(l >> 2) * 4][9][0] != 0) else 0) for l in L]
                    elif i == 10: res = [(inp.varin[l][10][sc] if sc < 2 else (0 if sc == 2 else ONE)) for l in L]
                    else: res = [0] * 16
                    uops += 1
                else:
                    res = [inp.varin[l][i][sc] for l in L]; uops += 1
                write(c, res)
        elif name == 'TEX':
            if inp.tex_fn is not None:
                for q in range(4):
                    if not (active >> (4 * q)) & 0xF: continue       # a quad without active lanes writes nothing
                    coords = [0] * 16
                    cm = d['cm'] or 0xF                                   # the coordinate set: unread components are 0
                    for p in range(4):
                        for k in range(4):
                            if (cm >> k) & 1: coords[p * 4 + k] = gpr[d['rs0']][4 * q + p][swz_comp(d['swz0'], k)]
                    tx = inp.tex_fn(d['samp'], d['mode'], coords)
                    for p in range(4):
                        l = 4 * q + p
                        if (active >> l) & 1:
                            for c in comps: gpr[rd][l][c] = tx[p * 4 + c]
            # one micro-op per coordinate of the set, at least two (they carry the checks for earlier writers of rd); the
            # core's TEX2 option (one per pair x y / z w) is off
            uops += max(bin(d['cm'] or 0xF).count('1'), 2) + len(comps)
        elif name == 'STO':
            a = [src(0, c) for c in range(4)]
            dst = psize if (vs and d['idx'] == 1) else outc         # STO 1 = the gl_PointSize row (vertex flavour)
            for l in L:
                if (active >> l) & 1 and not (killed >> l) & 1:
                    for c in range(4): dst[l][c] = a[c][l]
            uops += 4 if vs else 2                                  # fragment core: two components per micro-op
        elif name == 'STV':
            a = [src(0, c) for c in range(4)]
            for l in L:
                if (active >> l) & 1:
                    for c in range(4): outv[d['idx']][l][c] = a[c][l]
            uops += 4
        elif name == 'STOZ':
            a = src(0, 0); wrote_z = True
            for l in L:
                if (active >> l) & 1 and not (killed >> l) & 1: outz[l] = a[l]
            uops += 1
        elif name == 'SETP':
            a = src(0, 0); b = src(1, 0); pred = 0
            for l in L:
                if sr._setp(a[l], b[l], d['cc'], FMT): pred |= 1 << l
            uops += 1
        elif name == 'DISCARD': killed |= active; uops += 1
        elif name == 'IF':
            if len(mstack) >= IF_DEPTH: err = True; break
            then = active & pred; mstack.append((active, then)); active = then
            if active == 0: nxt = d['imm16']
            uops += 1
        elif name == 'ELSE':
            if not mstack: err = True; break
            ent, then = mstack[-1]; ll = (lstack[-1][3] & ~lstack[-1][4]) if lstack else 0xFFFF
            active = ent & ~then & ll & 0xFFFF
            if active == 0: nxt = d['imm16']
            uops += 1
        elif name == 'ENDIF':
            if not mstack: err = True; break
            ent = mstack.pop()[0]; ll = (lstack[-1][3] & ~lstack[-1][4]) if lstack else 0xFFFF
            active = ent & ll & 0xFFFF; uops += 1
        elif name == 'LOOP':
            if len(lstack) >= LOOP_DEPTH: err = True; break
            lstack.append([d['imm16'], pc + 1, active, active, 0]); uops += 1
        elif name == 'BREAK':
            if not lstack: err = True; break
            b = active & pred; lstack[-1][3] &= ~b & 0xFFFF; active &= ~b & 0xFFFF; uops += 1
        elif name == 'CONT':
            if not lstack: err = True; break
            c = active & pred; lstack[-1][4] |= c; active &= ~c & 0xFFFF; uops += 1
        elif name == 'ENDL':
            if not lstack: err = True; break
            lstack[-1][0] -= 1
            if lstack[-1][0] > 0 and lstack[-1][3] != 0:
                active = lstack[-1][3]; lstack[-1][4] = 0; nxt = lstack[-1][1]
            else:
                active = lstack[-1][2]; lstack.pop()
            uops += 1
        elif name == 'BRA': nxt = d['imm16']; uops += 1
        elif name == 'END': break
        elif name == 'NOP': uops += 1
        else: raise ValueError(f"simulate_fs2: op {op} at {pc}")
        pc = nxt
    if err and not vs: killed = 0xFFFF
    return FSResult(gpr, outc, killed, outz if wrote_z else None, err, steps, uops, outv, psize)


# ================================================================ vertex flavour (the same warp model: 16 vertices)
def simulate_vs2(words, inp, units=None, max_steps=200000):
    """The vertex core (shader_core_vs2): the warp simulator with lanes = vertices. inp.varin[l][i]
    = attribute i of vertex l (interp False), inp.params 256 vec4, inp.active the vertices present. Returns FSResult:
    gpr, outc = gl_Position per lane, psize = the STO 1 row per lane, outv[i][lane] = varying i, err, uops."""
    assert not inp.interp, "simulate_vs2: attributes are read with interp=False"
    return simulate_fs2(words, inp, units, max_steps, vs=True)


def default_vs_inputs(active=0xFFFF, rich=False):
    """Deterministic vertex inputs: params[i] broadcast (256 slots; rich: every slot AND component its own value, for
    programs that index the uniforms at run time — LDPX); attribute i of lane l differs per lane and component, so
    IF / ELSE diverges across the vertices of a warp."""
    def fb(x): return fr.encode_real(F(x).limit_denominator(1 << 14), FMT)
    if rich: params = [[fb(i * 0.125 + c * 0.03125 + 0.25) for c in range(4)] for i in range(256)]
    else: params = [[fb((i % 7) * 0.25 + 0.5)] * 4 for i in range(256)]
    varin = [[[fb((((i + 1) * (l + 1) * 0.5 + c * 0.125) % 3.0) + 0.25) for c in range(4)] for i in range(NVARY)]
             for l in range(16)]
    return FSInputs(params, varin, None, None, None, active & 0xFFFF, False, None)


# ================================================================ golden sets for the benches
def _hex(v, n=(FMT.W + 3) // 4): return f"{v:0{n}x}"


def vec4hex(v):  return ''.join(_hex(v[3 - k]) for k in range(4))                   # w z y x (MSB first)
def lanehex(vv): return ''.join(vec4hex(vv[15 - l]) for l in range(16))              # lane 15 .. 0, each w z y x


def write_words(words, path):
    with open(path, 'w') as f:
        for w in words: f.write(f"{w:016x}\n")


def default_fs_inputs(interp=False):
    """Deterministic inputs (the pattern of shader_ref.main_fs, extended to 4 quads): params[i] broadcast,
    varin rows differ per lane so IF / ELSE diverges (quad q repeats the single-quad pattern shifted by q)."""
    def fb(x): return fr.encode_real(F(x).limit_denominator(1 << 14), FMT)
    params = [[fb((i % 7) * 0.25 + 0.5)] * 4 for i in range(NPARAM)]
    varin = [[[fb((((i + 1) * (p + 1) * 0.5 + c * 0.125 + (l >> 2) * 0.0625) % 3.0) + 0.25) for c in range(4)]
              for i in range(NVARY)] for l in range(16) for p in [l & 3]]
    for l in range(16):                                 # the structure b_render gives rows 9 / 10 (quads 1, 3 front-facing)
        varin[l][9] = [ONE if (l >> 2) & 1 else 0, 0, 0, 0]
        varin[l][10][2:] = [0, ONE]
    lam = [(fb(0.25 + 0.03125 * l), fb(0.5 - 0.015625 * l)) for l in range(16)]
    tstore = [[([fb(0.5 + 0.125 * i + 0.25 * c) for c in range(4)], [fb(0.0625 * (c + 1) * (s + 1)) for c in range(4)],
                [fb(-0.03125 * (c + 1) * (i + 1)) for c in range(4)]) for i in range(8)] for s in range(8)]
    return FSInputs(params, varin, lam, [1, 2, 3, 4], tstore, 0xFFFF, interp, tex_model)


def main_fs2(prog, exact_rcp=False, interp=True, active=0xFFFF, deep=False):
    """The golden set for tb_fs2. interp: LDV 0..7 through the triangle store (the core always does; False = the
    single-board varin semantics). deep: assemble a program that overflows a stack on purpose."""
    name = os.path.splitext(os.path.basename(prog))[0]
    words, _ = assemble(open(prog).read().splitlines(), max_if=None if deep else IF_DEPTH, max_loop=None if deep else LOOP_DEPTH)
    inp = default_fs_inputs(interp); inp.active = active & 0xFFFF
    r = simulate_fs2(words, inp, Units(exact_rcp))
    write_words(words, f"{GEN}/{name}_fs2_iram.mem")
    with open(f"{GEN}/{name}_fs2_params.mem", 'w') as f:
        for v in inp.params: f.write(vec4hex(v) + "\n")
    with open(f"{GEN}/{name}_fs2_varin.mem", 'w') as f:                  # row i (0..15): 16 lanes
        for i in range(NVARY): f.write(lanehex([inp.varin[l][i] for l in range(16)]) + "\n")
    with open(f"{GEN}/{name}_fs2_lam.mem", 'w') as f:                    # per lane {l2, l1}, lane 15 .. 0
        f.write(''.join(_hex(inp.lam[15 - l][1]) + _hex(inp.lam[15 - l][0]) for l in range(16)) + "\n")
    with open(f"{GEN}/{name}_fs2_tstore.mem", 'w') as f:                 # slot s, varying i: {D2, D1, f0}
        for s in range(8):
            for i in range(8):
                f0, d1, d2 = inp.tstore[s][i]; f.write(vec4hex(d2) + vec4hex(d1) + vec4hex(f0) + "\n")
    with open(f"{GEN}/{name}_fs2_tslot.mem", 'w') as f: f.write(''.join(f"{inp.tslot[3 - q]:x}" for q in range(4)) + "\n")
    with open(f"{GEN}/{name}_fs2_expgpr.mem", 'w') as f:
        for v in r.gpr: f.write(lanehex(v) + "\n")
    with open(f"{GEN}/{name}_fs2_outc.mem", 'w') as f: f.write(lanehex(r.outc) + "\n")
    with open(f"{GEN}/{name}_fs2_kill.mem", 'w') as f: f.write(f"{r.killed:04x}\n")
    with open(f"{GEN}/{name}_fs2_outz.mem", 'w') as f:                   # lane 15 .. 0; zeros when never written
        f.write(''.join(_hex(r.outz[15 - l] if r.outz is not None else 0) for l in range(16)) + "\n")
    with open(f"{GEN}/{name}_fs2_outzw.mem", 'w') as f: f.write(("1" if r.outz is not None else "0") + "\n")
    with open(f"{GEN}/{name}_fs2_err.mem", 'w') as f: f.write(("1" if r.err else "0") + "\n")
    print(f"{name} (FS2, {FMT.name}, interp={int(interp)}): {len(words)} instr, {r.uops} micro-ops, err={int(r.err)} "
          f"-> tools/gen/{name}_fs2_*.mem")
    return r


def main_vs2(prog, exact_rcp=False, active=0xFFFF, deep=False):
    """The golden set for tb_vs2: iram (64-bit words), params (256 vec4), attr (attribute i = 16 lanes), expgpr
    (16 lanes x 4 components per register), outpos / outpsz (16 lanes), outv (varying i = 16 lanes), err."""
    name = os.path.splitext(os.path.basename(prog))[0]
    words, _ = assemble(open(prog).read().splitlines(), vs=True, max_if=None if deep else IF_DEPTH,
                        max_loop=None if deep else LOOP_DEPTH)
    inp = default_vs_inputs(active, rich=any(decode(w)['op'] == OPS['LDPX'] for w in words))
    r = simulate_vs2(words, inp, Units(exact_rcp))
    write_words(words, f"{GEN}/{name}_vs2_iram.mem")
    with open(f"{GEN}/{name}_vs2_params.mem", 'w') as f:
        for v in inp.params: f.write(vec4hex(v) + "\n")
    with open(f"{GEN}/{name}_vs2_attr.mem", 'w') as f:                   # attribute i (0..15): 16 lanes, lane 15 first
        for i in range(NVARY): f.write(lanehex([inp.varin[l][i] for l in range(16)]) + "\n")
    with open(f"{GEN}/{name}_vs2_expgpr.mem", 'w') as f:
        for v in r.gpr: f.write(lanehex(v) + "\n")
    with open(f"{GEN}/{name}_vs2_outpos.mem", 'w') as f: f.write(lanehex(r.outc) + "\n")
    with open(f"{GEN}/{name}_vs2_outpsz.mem", 'w') as f: f.write(lanehex(r.psize) + "\n")
    with open(f"{GEN}/{name}_vs2_outv.mem", 'w') as f:
        for i in range(NVARY): f.write(lanehex(r.outv[i]) + "\n")
    with open(f"{GEN}/{name}_vs2_err.mem", 'w') as f: f.write(("1" if r.err else "0") + "\n")
    print(f"{name} (VS2, {FMT.name}): {len(words)} instr, {r.uops} micro-ops, err={int(r.err)} -> tools/gen/{name}_vs2_*.mem")
    return r


# ================================================================ self-test against the 32-bit tools
def _old_fs_inputs_to_new(params, varys):
    """shader_ref.main_fs layout -> FSInputs with interp=False: quad 0 = shader_ref's 4 pixels, quads 1..3 copies."""
    varin = [[[varys[i][p * 4 + c] for c in range(4)] for i in range(NVARY)] for l in range(16) for p in [l & 3]]
    return FSInputs(params, varin, None, None, None, 0xFFFF, False)


def _tainted(words):
    """Registers whose value depends on ops the 32-bit reference simulator does not implement (DDX, DDY, TEX),
    the STO / STOZ sources among them, by a fixed-point taint pass (control flow ignored: conservative)."""
    t = set(); outc_t = False; outz_t = False
    for _ in range(len(words) + 1):
        before = (len(t), outc_t, outz_t)
        for w in words:
            d = decode(w); name = OPNAME.get(d['op'], '?')
            if d['op'] == 21: name = 'STOZ'
            if d['op'] == OPS['IOP']: name = 'IOP'
            srcs = {d['rs0'], d['rs1'], d['rs2']} if name in ('MAD', 'SEL') else {d['rs0'], d['rs1']} \
                if (name in ARITH2 or name in INT2 or name in ('IOP', 'SET')) else {d['rs0']} if name in ARITH1 + ('STO', 'STOZ', 'TEX') else set()
            if name in ('DDX', 'DDY', 'TEX') or (name in WRITES and srcs & t): t.add(d['rd'])
            if name == 'STO' and d['rs0'] in t: outc_t = True
            if name == 'STOZ' and d['rs0'] in t: outz_t = True
        if (len(t), outc_t, outz_t) == before: break
    return t, outc_t, outz_t


def _semantic_checks():
    """Hand-computed results for what the 32-bit ISA cannot express: swizzles, MOVI, chains on swizzled sources,
    modifiers on every source, 16-lane divergence, partial warps, interpolation, quad derivatives."""
    def fb(x): return fr.encode_real(F(x), FMT)
    def rv(v): return fr.to_real(v, FMT)
    def run(lines, inp=None, **kw): return simulate_fs2(assemble(lines, **kw)[0], inp or FSInputs())
    r = run(["MOVI r1.x, #1", "MOVI r1.y, #2", "MOVI r1.z, #3", "MOVI r1.w, #4",
             "MOV r2, r1.wzyx", "DP4 r3.x, r1, r1.yxwz", "DP3 r3.yz, r1.zzz, r1.w", "MAD r4, -r1.x, r1, !r2.w",
             "ADD r5.xw, r1.w, r1.x", "SEL r6, r1.x, r2, r1", "SUB r7.y, r2.x, -r1.w", "STO r2.yyxx", "END"])
    g = r.gpr
    assert [rv(v) for v in g[1][5]] == [1, 2, 3, 4] and [rv(v) for v in g[2][0]] == [4, 3, 2, 1]
    assert rv(g[3][0][0]) == 28 and rv(g[3][0][1]) == 36 and rv(g[3][0][2]) == 36 and g[3][0][3] == 0
    assert [rv(v) for v in g[4][9]] == [0, -1, -2, -3]            # -r1.x * r1 + |r2.w| = -1 * r1 + 1
    assert [rv(v) for v in g[5][0]] == [5, 0, 0, 5] and [rv(v) for v in g[6][0]] == [4, 3, 2, 1]
    assert rv(g[7][0][1]) == 8 and [rv(v) for v in r.outc[15]] == [3, 3, 4, 4]
    assert r.uops == 4 + 4 + 5 + (4 + 1) + 4 + 2 + 4 + 1 + 2 + 0, r.uops   # micro-op accounting (STO: 2)
    # divergence across the 16 lanes, kill, partial warp, STOZ with a swizzle
    inp = FSInputs(interp=False, active=0x0FFF)
    for l in range(16): inp.varin[l][8] = [fb(l), fb(0.5), fb(l % 3), fb(1)]
    r = run(["LDV r1, 8", "MOVI r2.x, #7.5", "SETP SLT, r1.x, r2.x", "IF L1", "MOVI r3, #1", "L1:", "ELSE L2",
             "MOVI r3, #2", "L2:", "ENDIF", "MOVI r9.x, #1", "SETP SEQ, r1.z, r9.x", "IF L3", "DISCARD", "L3:",
             "ENDIF", "STOZ r1.zxyw", "STO r3", "END"], inp)
    for l in range(16):                                     # inactive (12..15) and killed (l % 3 == 1) lanes: no STO
        exp = 0 if (l >= 12 or l % 3 == 1) else (1 if l < 7.5 else 2)
        assert rv(r.outc[l][0]) == exp and rv(r.outc[l][3]) == exp, (l, r.outc[l])
    assert r.killed == sum(1 << l for l in range(12) if l % 3 == 1), f"{r.killed:04x}"
    assert all(rv(r.outz[l]) == l % 3 for l in range(12) if l % 3 != 1) and r.outz[13] == 0
    # LDV interpolation (triangle store, per-quad slots), derivatives inside quads, LDP with a swizzle
    inp = FSInputs(interp=True)
    inp.tslot = [2, 2, 5, 5]
    inp.tstore[2][3] = ([fb(1), fb(2), fb(3), fb(4)], [fb(0)] * 4, [fb(0)] * 4)          # D = 0: exact f0
    inp.tstore[5][3] = ([fb(0)] * 4, [fb(10), fb(20), fb(30), fb(40)], [fb(1)] * 4)       # l1 * D1 + l2
    inp.lam = [(fb(1), fb(0))] * 8 + [(fb(0.5), fb(2))] * 8
    inp.params[7] = [fb(9), fb(8), fb(7), fb(6)]
    r = run(["LDV r1, 3", "LDV r2.x, 3.w", "LDP r3.yw, 7.zx", "LDP r4.yw, 7.wzyx", "LDV r5, 8", "END"], inp)
    assert [rv(v) for v in r.gpr[1][3]] == [1, 2, 3, 4] and [rv(v) for v in r.gpr[1][12]] == [7, 12, 17, 22]
    assert rv(r.gpr[2][2][0]) == 4 and rv(r.gpr[2][9][0]) == 22
    # the swizzle is indexed by the destination component: .zx = z x x x, so .y and .w both read params[7].x
    assert [rv(v) for v in r.gpr[3][0]] == [0, 9, 0, 9] and [rv(v) for v in r.gpr[4][0]] == [0, 7, 0, 9]
    inp = FSInputs(interp=False)
    for l in range(16): inp.varin[l][8] = [fb(1 + (l & 3)), fb(10 * (l >> 2)), 0, 0]
    r = run(["LDV r1, 8", "DDX r2.x, r1", "DDY r2.y, r1.x", "DDX r3, r1.yxxx", "END"], inp)
    assert all(rv(r.gpr[2][l][0]) == 1 and rv(r.gpr[2][l][1]) == 2 for l in range(16))
    assert all(rv(r.gpr[3][l][0]) == 0 and rv(r.gpr[3][l][1]) == 1 for l in range(16))
    # TEX with the bench's texture model: a swizzled coordinate, a partial mask, the destination = the coordinate
    # register (read before any write), inactive lanes untouched; A carries sampler and mode
    inp = FSInputs(interp=False, active=0xFFF0, tex_fn=tex_model)
    for l in range(16): inp.varin[l][8] = [fb(l), fb(0.5), fb(l % 3), fb(2)]
    r = run(["LDV r1, 8", "MOVI r3, #0", "TEX r2, r1.wzyx, #5, LOD", "TEX r3.xz, r1, #0", "TEX r1, r1, #2", "END"], inp)
    # r2: s = r1.w = 2, t = r1.z = l % 3, r = r1.y = 0.5, q = r1.x = l -> R 2, G t's exponent with l's mantissa, B 1, A 23
    assert all(rv(r.gpr[2][l][0]) == 2 and rv(r.gpr[2][l][2]) == 1 and rv(r.gpr[2][l][3]) == 23 for l in range(4, 16))
    assert r.gpr[2][6][1] == 0 and rv(r.gpr[2][4][1]) == 1 and rv(r.gpr[2][7][1]) == F(7, 4) and rv(r.gpr[2][11][1]) == F(11, 4)
    # r3.xz: the texel components x and z land, y and w stay (the coordinate is the whole swizzled rs0)
    assert all(rv(r.gpr[3][l][0]) == l and rv(r.gpr[3][l][2]) == 2 * (l % 3) and r.gpr[3][l][1] == 0 and r.gpr[3][l][3] == 0 for l in range(4, 16))
    # r1 = TEX of itself: read before any write
    assert all(rv(r.gpr[1][l][0]) == l and rv(r.gpr[1][l][1]) == F(1, 2) and rv(r.gpr[1][l][2]) == 2 * (l % 3) and rv(r.gpr[1][l][3]) == 9 for l in range(4, 16))
    assert all(r.gpr[k][l] == [0, 0, 0, 0] for k in (1, 2, 3) for l in range(4))     # inactive lanes: nothing written
    assert rv(tex_model(3, 2, [0] * 16)[3]) == 15 and tex_model(0, 0, [0] * 16)[1] == 0
    # vertex flavour: 16 vertices per warp with their own attributes, IF / ELSE divergence across them, STV, STO 0 / 1,
    # a swizzled LDP from the 256 uniforms; the fragment-only ops and BRP are rejected in a vertex program
    inp = default_vs_inputs()
    for l in range(16): inp.varin[l][0] = [fb(l), fb(2), fb(3), fb(1)]
    r = simulate_vs2(assemble(["LDV r1, 0", "MOVI r3.x, #7.5", "SETP SLT, r1.x, r3.x", "IF L1", "MOVI r4, #1", "L1:",
                               "ELSE L2", "MOVI r4, #2", "L2:", "ENDIF", "DP4 r5.x, r1, r1.yxyx", "LDP r6, 3.wzyx",
                               "STV 3, r4", "STV 15, r1.wzyx", "STO r5.x", "STO 1, r1.yyyy", "END"], vs=True)[0], inp)
    for l in range(16):
        assert rv(r.outv[3][l][0]) == (1 if l < 7.5 else 2) and [rv(v) for v in r.outv[15][l]] == [1, 3, 2, l], l
        assert rv(r.outc[l][0]) == 5 * l + 6 and rv(r.psize[l][2]) == 2, l            # (l,2,3,1).(2,l,2,l) = 5 l + 6
        assert r.gpr[6][l] == [inp.params[3][3 - c] for c in range(4)] and r.outv[0][l] == [0] * 4
    assert not r.err and r.uops == 4 + 1 + 1 + 1 + 4 + 1 + 4 + 1 + 5 + 4 + 4 + 4 + 4 + 4, r.uops
    for bad in (["BRP 3"], ["TEX r1, r2, #0"], ["STOZ r1"], ["DISCARD"], ["DDX r1, r2"], ["STO 2, r1"], ["STV 16, r1"]):
        try: assemble(bad, vs=True); raise AssertionError("accepted in a vertex program: " + bad[0])
        except AsmError: pass
    try: assemble(["STV 0, r1"]); raise AssertionError("STV accepted in a fragment program")
    except AsmError: pass
    # LDPX: a per-lane gather from the uniforms — lane l's index = l - 2 (negative wraps), the swizzled value
    # component, a non-splat index swizzle (component c reads index component c), a masked write, the 256 wrap
    inp = default_vs_inputs(rich=True)
    for l in range(16): inp.varin[l][0] = [fb(l - 2), fb(2 * l), fb(0.75), fb(255)]
    r = simulate_vs2(assemble(["LDV r1, 0", "FTOI r2, r1", "LDPX r3, 10, r2.x", "LDPX r4.yw, 7.wzyx, r2.y", "LDPX r5, 0, r2",
                               "LDPX r6, 4, r2.w", "MOVI r7, #1", "END"], vs=True)[0], inp)
    for l in range(16):
        assert r.gpr[3][l] == inp.params[(10 + l - 2) & 0xFF], (l, r.gpr[3][l])
        assert r.gpr[4][l] == [0, inp.params[7 + 2 * l][2], 0, inp.params[7 + 2 * l][0]], l    # .wzyx: y reads z, w reads x
        assert r.gpr[5][l] == [inp.params[l - 2 & 0xFF][0], inp.params[2 * l][1], inp.params[0][2], inp.params[255][3]], l
        assert r.gpr[6][l] == inp.params[(4 + 255) & 0xFF], l                                   # 259 wraps to 3
    assert r.uops == 4 + 4 + 4 + 2 + 4 + 4 + 4, r.uops
    assert disasm(assemble(["LDPX r4.yw, 7.wzyx, r2.y"], vs=True)[0][0]) == "LDPX r4.yw, 7.wzyx, r2.y"
    for bad in (["LDPX r1, 256, r2.x"], ["LDPX r1, 3, -r2.x"], ["LDPX r2.xy, 3, r2.x"]):   # the last: .y reads the .x just written
        try: assemble(bad, vs=True); raise AssertionError("accepted: " + bad[0])
        except AsmError: pass
    assemble(["LDPX r2.x, 3, r2.x", "LDPX r2.xy, 3, r2.y"], vs=True)      # a component is read before it is written
    try: assemble(["LDPX r1, 3, r2.x"]); raise AssertionError("LDPX accepted in a fragment program")
    except AsmError: pass
    print("semantic checks: ok")


def selftest(tests_dir):
    import glob
    n_ok = 0; n_fail = 0
    _semantic_checks()
    # --- unit checks of the encoding
    assert enc_swz('xyzw') == SWZ_ID and enc_swz('x') == 0 and enc_swz('wzyx') == 0x1B and enc_swz('xy') == 0x54
    w = encode(OPS['MAD'], 0x5, 3, (1, enc_swz('wzyx'), 1), (2, SWZ_ID, 2), (31, enc_swz('y'), 3))
    dd = decode(w); assert (dd['op'], dd['wm'], dd['rd'], dd['rs0'], dd['swz0'], dd['m0'], dd['rs1'], dd['m1'],
                            dd['rs2'], dd['swz2'], dd['m2']) == (6, 5, 3, 1, 0x1B, 1, 2, 2, 31, 0x55, 3)
    assert disasm(assemble(["MAD r3.xz, -r1.wzyx, !r2, -!r31.y"])[0][0]) == "MAD r3.xz, -r1.wzyx, !r2, -!r31.y"
    assert disasm(assemble(["MOVI r4.w, #1.5"])[0][0]) == "MOVI r4.w, #0x3f8000"
    assert disasm(assemble(["LDP r2, 7.zw"])[0][0]) == "LDP r2, 7.zwww"
    assert disasm(assemble(["TEX r6, r1.xyzw, #3, LOD"])[0][0]) == "TEX r6, r1, #3, LOD"
    for bad in (["MOV r1.xy, r1.yx"], ["ADD r2.xyz, r3, r2.zzx"], ["LDP r1, 64"]):
        try: assemble(bad); raise AssertionError("accepted: " + bad[0])
        except AsmError: pass
    assemble(["MOV r1.xy, r1.xy", "ADD r1.x, r1.y, r1.z", "MAD r1.yz, r1.yz, r1.z, r1.w", "DP4 r1, r1.wzyx, r1"])
    print("encoding / checker: ok")
    # --- every test program: old vs new simulators (identical lanes), old vs new encoders (fields agree)
    for path in sorted(glob.glob(os.path.join(tests_dir, "*.sasm"))):
        name = os.path.splitext(os.path.basename(path))[0]
        lines = open(path).read().splitlines()
        is_fs = name.startswith(('fs_', 'test_fs', 'b_fs')) or name.endswith('_fs')
        try:
            if name.startswith('fs2_'):                      # 64-bit-only programs: assemble + simulate (no 32-bit twin)
                deep = name.startswith('fs2_stk')
                words_new, _ = assemble(lines, max_if=None if deep else IF_DEPTH, max_loop=None if deep else LOOP_DEPTH)
                r = simulate_fs2(words_new, default_fs_inputs(interp=True))
                assert r.err == deep, f"{name}: stack violation {r.err}"
                n_ok += 1; print(f"  ok   {name} (FS2 only, {len(words_new)} instr, {r.uops} micro-ops)"); continue
            if name.startswith('vs2_'):                      # 64-bit-only vertex programs (LDPX): assemble + simulate
                words_new, _ = assemble(lines, vs=True, max_if=IF_DEPTH, max_loop=LOOP_DEPTH)
                r = simulate_vs2(words_new, default_vs_inputs(rich=True))
                assert not r.err, f"{name}: unexpected stack violation"
                n_ok += 1; print(f"  ok   {name} (VS2 only, {len(words_new)} instr, {r.uops} micro-ops)"); continue
            deep = name.startswith('test_fsstk')
            if not is_fs and any(re.match(r'\s*BRP\b', ln.split('//')[0], re.I) for ln in lines):
                n_ok += 1; print(f"  ok   {name} (32-bit vertex core only: BRP is not a 64-bit vertex instruction)"); continue
            words_old, _ = sr.assemble(lines)
            words_new, _ = assemble(lines, vs=not is_fs, max_if=None if deep else IF_DEPTH)
            assert len(words_old) == len(words_new)
            taint, outc_t, outz_t = _tainted(words_new)
            for wo, wn in zip(words_old, words_new):
                opo = (wo >> 26) & 0x3F; dn = decode(wn)
                assert opo == dn['op'], f"{name}: opcode {opo} vs {dn['op']}"
            if is_fs:
                def fb(x): return fr.encode_real(F(x).limit_denominator(1 << 14), FMT)
                params = [[fb((i % 7) * 0.25 + 0.5)] * 4 for i in range(sr.NPARAM)]
                varys = [[fb(((i + 1) * (p + 1) * 0.5 + c * 0.125) % 3.0 + 0.25) for p in range(4) for c in range(4)]
                         for i in range(NVARY)]
                g_old, outc_old, kill_old, z_old = sr.simulate_simt(words_old, params, varys, FMT, want_depth=True)
                r = simulate_fs2(words_new, _old_fs_inputs_to_new(params, varys))
                if r.err:                                   # shader_ref's simulator has no stack limits (tb_fs_ctx +EXPERR)
                    assert name.startswith('test_fsstk'), f"{name}: unexpected stack violation"
                    n_ok += 1; print(f"  ok   {name} (FS, stack violation detected as expected)"); continue
                for rg in range(NREG):
                    if rg in taint: continue                # shader_ref's simulator has no DDX / DDY / TEX
                    for p in range(4):
                        for c in range(4):
                            assert r.gpr[rg][p][c] == g_old[rg][p * 4 + c], f"{name}: r{rg} pixel {p} .{'xyzw'[c]}"
                if not outc_t:
                    for p in range(4):
                        for c in range(4): assert r.outc[p][c] == outc_old[p * 4 + c], f"{name}: outc pixel {p}"
                assert (r.killed & 0xF) == kill_old, f"{name}: kill {r.killed:x} vs {kill_old:x}"
                if z_old is not None:
                    if not outz_t:
                        for p in range(4): assert r.outz[p] == z_old[p], f"{name}: outz pixel {p}"
                else: assert r.outz is None
                # quads 1..3 got the same inputs as quad 0: identical results there too
                for rg in range(NREG):
                    for l in range(4, 16):
                        assert r.gpr[rg][l] == r.gpr[rg][l & 3], f"{name}: quad {l >> 2} differs from quad 0 (r{rg})"
                if taint: print(f"       ({name}: {len(taint)} registers from DDX / DDY / TEX not compared)")
            else:
                def fb(x): return fr.encode_real(F(x).limit_denominator(1 << 20), FMT)
                params = [[fb((i % 7) * 0.25 + 0.5)] * 4 for i in range(256)]
                varys = [[fb((i % 5) * 0.5 + 0.25)] * 4 for i in range(NVARY)]
                g_old, pos_old, outv_old = sr.simulate(words_old, params, varys)
                # the warp simulator with the same attributes on every lane: every vertex equals the single-vertex result
                r = simulate_vs2(words_new, FSInputs(params, [[list(v) for v in varys] for _ in range(16)], interp=False))
                assert not r.err, f"{name}: unexpected stack violation"
                for rg in range(NREG):
                    for l in range(16): assert r.gpr[rg][l] == g_old[rg], f"{name}: r{rg} lane {l}"
                for l in range(16):
                    assert r.outc[l] == pos_old, f"{name}: position lane {l}"
                    for i in range(NVARY): assert r.outv[i][l] == outv_old[i], f"{name}: varying {i} lane {l}"
            n_ok += 1; print(f"  ok   {name} ({'FS' if is_fs else 'VS'}, {len(words_new)} instr)")
        except Exception as e:
            n_fail += 1; print(f"  FAIL {name}: {e}")
    print(f"selftest: {n_ok} ok, {n_fail} failed")
    return n_fail == 0


def main():
    a = sys.argv[1:]
    if not a or a[0] in ('-h', '--help'): print(__doc__); return
    cmd = a[0]
    if cmd == 'asm':
        words, _ = assemble(open(a[1]).read().splitlines(), vs='--vs' in a)
        out = a[a.index('-o') + 1] if '-o' in a else f"{GEN}/{os.path.splitext(os.path.basename(a[1]))[0]}_fs2_iram.mem"
        write_words(words, out); print(f"{len(words)} instr -> {out}")
    elif cmd == 'dis':
        vs = '--vs' in a
        for i, ln in enumerate(open(a[1])):
            ln = ln.strip()
            if ln: print(f"{i:4d}  {disasm(int(ln, 16), vs)}")
    elif cmd == 'fs2':                                   # [--exact-rcp] [--no-interp] [--active HEX] [--deep]
        act = int(a[a.index('--active') + 1], 16) if '--active' in a else 0xFFFF
        main_fs2(a[1], '--exact-rcp' in a, '--no-interp' not in a, act, '--deep' in a)
    elif cmd == 'vs2':                                   # [--exact-rcp] [--active HEX] [--deep]
        act = int(a[a.index('--active') + 1], 16) if '--active' in a else 0xFFFF
        main_vs2(a[1], '--exact-rcp' in a, act, '--deep' in a)
    elif cmd == 'selftest':
        ok = selftest(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'tests'))
        sys.exit(0 if ok else 1)
    else: print(__doc__); sys.exit(1)


if __name__ == "__main__": main()
