#!/usr/bin/env python3
"""gpu2 shader assembler + reference simulator.

Assembles a tiny shader ISA to iram words and simulates it (using the validated
float_ref FP ops) to produce the expected final GPR state — the golden for
tb_shader_core. Operates on one vec4 (4 components/lanes), FP24 by default.

ISA (32-bit):  op[31:26] rd[25:21] rs0[20:16] rs1[15:11] rs2[10:8] sub[7:4] pf[3:0]
  imm16 = bits[15:0]  (LD/ST/BRA/BRP/LOOP).  BRP branch-if-false flag = ins[25] (unused rd field,
  outside imm16 so it can't corrupt the branch target).

Ops: NOP MOV ADD SUB MUL MAD MIN MAX DP3 DP4 RCP RSQRT LOG2 EXP2 SETP
     LDP(load param) LDV(load varying) STV(store varying) STO(store out)
     BRA BRP LOOP ENDL END
Run: python3 tools/shader_ref.py <prog.sasm>  -> tools/gen/<prog>_{iram,params,varying,expgpr}.mem
"""
import os, sys, re
from fractions import Fraction as F
sys.path.insert(0, os.path.dirname(__file__))
import float_ref as fr
def _sr(bits,FMT):
    v=fr.to_real(bits,FMT)
    return v if v is not None else (1e30 if not (bits>>(FMT.W-1)) else -1e30)
import math as _math
def _flog2(x):  return _math.log2(x) if x>0 else -1e30       # RTL SFU: log2(<=0) -> -inf (don't crash)
def _frsqrt(x): return x**-0.5 if x>0 else 1e30             # rsqrt(<=0) -> +inf-ish (unused/edge lanes)
def _frcp(x):   return 1.0/x if x!=0 else 1e30
def _fexp2(x):
    try: return 2.0**x
    except OverflowError: return 1e30
def _enc_sfu(v, FMT):
    """A unit's result (a Python float) as the format's bits. F(v).limit_denominator(2^28) keeps the fractions short, but
    would turn a value below 2^-29 into 0 and lose bits the format holds below 2^-8: a small value is encoded exactly,
    the others through limit_denominator."""
    f = F(v)
    return fr.encode_real(f if abs(v) < 2.0 ** -8 else f.limit_denominator(1 << 28), FMT)
def _setp(ab,bb,sub,FMT):
    """SETP predicate for one lane, matching float_cmp.sv: sub = CMP sub-op from ins[7:4]
       (SLT=2,SGE=3,SEQ=4,SNE=5). Returns 1/0."""
    a=_sr(ab,FMT); b=_sr(bb,FMT)
    if   sub==2: return 1 if a <  b else 0   # SLT
    elif sub==3: return 1 if a >= b else 0   # SGE
    elif sub==4: return 1 if a == b else 0   # SEQ
    else:        return 1 if a != b else 0   # SNE (5)

GEN = os.path.join(os.path.dirname(__file__), "gen"); os.makedirs(GEN, exist_ok=True)
FMT = fr.FP24                      # core under test is FP24 VPU
NREG, NPARAM, NVARY = 32, 256, 16

OPS = dict(NOP=0,MOV=1,MOVI=2,ADD=3,SUB=4,MUL=5,MAD=6,MIN=7,MAX=8,DP3=9,DP4=10,
           RCP=11,RSQRT=12,LOG2=13,EXP2=14,SETP=15,LDP=16,LDV=17,STV=18,STO=19,
           BRA=20,BRP=21,LOOP=22,ENDL=23,IF=24,ELSE=25,ENDIF=26,
           BREAK=27,CONT=28,DISCARD=29,FTOI=30,ITOF=31,
           STOZ=21,DDX=32,DDY=33,TEX=34,   # FS-only ops (STOZ shares opcode 21 with BRP; FS core has no BRP)
           SET=35,                         # exact GPR-writing compare: rd = float_cmp(ra,rb,sub) = 1.0/0.0
           IADD=36,ISUB=37,ISLT=38,ISGE=39, # integer ALU (2's-comp, latency 1): low-overhead loop counters
           SIN=40,COS=41,                   # sin/cos via the SFU quarter-wave LUT (full-range)
           TRUNC=42,FLR=43,CEIL=44,RNDE=45, # float -> float rounding (float_round, exact, latency 2)
           SEL=46,                          # SEL rd, rc, ra, rb: per lane rc != 0 ? ra : rb (exact select, latency 1)
           IMUL=47,IDIV=48,                 # integer ALU (int_unit): low W bits of a*b; a/b toward 0, x/0 = -1
           ISEQ=49,ISNE=50,                 # exact integer ==, != -> 1.0/0.0 (1-cycle integer slot)
           IOP=51,                          # integer logic / shift / min / max / unsigned compare: fn = ins[3:0]
           IMOD=52,UDIV=53,UMOD=54,         # divider: signed remainder (sign of the dividend), unsigned / and %
           FTOU=55,UTOF=56,                 # float <-> unsigned W-bit (FTOU: negative -> 0, saturate 2^W - 1)
           END=63)
RND_OPS = ('TRUNC','FLR','CEIL','RNDE')     # rounding mode = opcode - TRUNC (float_ref.fround)
# IOP functions (assembler mnemonics -> ins[3:0]); INOT / IABS / INEG / ISIGN read rs0 only
IOP_FN = dict(IAND=0,IOR=1,IXOR=2,INOT=3,ISHL=4,ISHR=5,USHR=6,IMIN=7,IMAX=8,UMIN=9,UMAX=10,IABS=11,INEG=12,ISIGN=13,
              ULT=14,UGE=15)

def sel_rb(w): return ((w>>8)&7) | (((w>>2)&3)<<3)  # SEL / MAD third register: ins[10:8] + ins[3:2] (5 bits)
def sel_lane(c, a, b, FMT):                          # c != 0.0 (exponent field nonzero: FTZ, NaN counts) ? a : b
    return a if ((c >> FMT.M) & ((1<<FMT.E)-1)) != 0 else b

def ftoi(bits, FMT):
    # float -> signed int (truncate toward zero), W-bit two's complement
    r=fr.to_real(bits,FMT); iv=int(r) if r is not None else 0      # int() truncates toward 0
    iv=max(-(1<<(FMT.W-1)), min((1<<(FMT.W-1))-1, iv))
    return iv & ((1<<FMT.W)-1)
def ftou(bits, FMT):
    # float -> unsigned W-bit (truncate toward zero), negative -> 0, saturate at 2^W - 1; Inf / NaN -> 0
    r=fr.to_real(bits,FMT); iv=int(r) if r is not None else 0
    return max(0, min((1<<FMT.W)-1, iv))
def utof(bits, FMT):
    return fr.encode_real(F(bits & ((1<<FMT.W)-1)), FMT)
def itof(bits, FMT):
    v=bits & ((1<<FMT.W)-1)
    if v>=(1<<(FMT.W-1)): v-=(1<<FMT.W)
    return fr.encode_real(F(v), FMT)
CMP = dict(SLT=2,SGE=3,SEQ=4,SNE=5)     # SETP sub-op

def reg(t):  return int(t[1:]) if t and t[0] in 'rR' else int(t)

# operand source modifier from a prefix: '-' negate, '!' abs, '-!' -abs. Returns (regnum, mod2bit).
def regmod(t):
    mod=0
    while t and t[0] in '-!':
        if t[0]=='-': mod^=1        # toggle negate
        elif t[0]=='!': mod|=2      # abs
        t=t[1:]
    # '-!' => negate+abs => -abs (mod 3); '-' => 1; '!' => 2
    return reg(t), (mod & 3)

# component write-mask suffix on the mnemonic: "ADD.xy" -> bits x=0,y=1,z=2,w=3 -> ins[7:4]. "" -> 0 (=full).
def maskbits(suf):
    b=0
    for c in suf.lower():
        b |= 1 << 'xyzw'.index(c)
    return b

HW_LOOP_DEPTH = 4      # shader_core LOOP stack (lsp 3-bit, lcnt/lpc arrays [0:3]); deeper nesting overflows

def assemble(lines):
    # pass 1: labels (+ guard runtime-LOOP nesting against the hardware stack depth so hand-written asm
    # fails loudly instead of silently overflowing lsp. The GLSL compiler unrolls loops -> emits no LOOP.)
    labels={}; pc=0; prog=[]; depth=0; maxd=0
    for ln in lines:
        ln=ln.split('//')[0].split(';')[0].strip()
        if not ln: continue
        if ln.endswith(':'): labels[ln[:-1]]=pc; continue
        mn0=re.split(r'[\s,]+', ln)[0].upper()
        if mn0=='LOOP': depth+=1; maxd=max(maxd,depth)
        elif mn0=='ENDL': depth-=1
        prog.append(ln); pc+=1
    if maxd>HW_LOOP_DEPTH:
        raise ValueError(f"LOOP nesting {maxd} exceeds hardware stack depth {HW_LOOP_DEPTH} "
                         f"(shader_core lsp) — would silently overflow; unroll or reduce nesting")
    words=[]; src=[]
    for ln in prog:
        t=re.split(r'[\s,]+', ln)
        mn_full=t[0].upper(); mn, _, msuf = mn_full.partition('.'); mn=mn
        op=OPS['IOP'] if mn in IOP_FN else OPS[mn]; w=op<<26
        wm = maskbits(msuf) if msuf else 0xF         # ins[7:4] write-mask; no suffix => full (1111)
        if mn in ('ADD','SUB','MUL','MIN','MAX','DP3','DP4'):
            r0,m0=regmod(t[2]); r1,m1=regmod(t[3])
            w|=reg(t[1])<<21|r0<<16|r1<<11|(wm<<4)|(m0<<8)|(m1<<0)
        elif mn in ('IADD','ISUB','ISLT','ISGE','ISEQ','ISNE','IMUL','IDIV','IMOD','UDIV','UMOD'):   # integer: rd, rs0, rs1
            w|=reg(t[1])<<21|reg(t[2])<<16|reg(t[3])<<11|(wm<<4)
        elif mn in IOP_FN:                           # IOP rd, rs0[, rs1]; function in ins[3:0]
            w=(OPS['IOP']<<26)|reg(t[1])<<21|reg(t[2])<<16|((reg(t[3]) if len(t)>3 and t[3] else 0)<<11)|(wm<<4)|IOP_FN[mn]
        elif mn=='MAD':                              # rs2 occupies [10:8] -> no src0 modifier for MAD
            r1,m1=regmod(t[3])
            r2=reg(t[4])                             # addend: any register (ins[10:8] + ins[3:2])
            w|=reg(t[1])<<21|reg(t[2])<<16|r1<<11|(r2&7)<<8|(wm<<4)|((r2>>3)&3)<<2|(m1<<0)
        elif mn in ('MOV','RCP','RSQRT','LOG2','EXP2','SIN','COS','FTOI','ITOF','FTOU','UTOF','DDX','DDY')+RND_OPS:
            r0,m0=regmod(t[2])
            w|=reg(t[1])<<21|r0<<16|(wm<<4)|(m0<<8)
        elif mn=='SEL':                              # SEL rd, rc, ra, rb (no source modifiers)
            rb=reg(t[4])
            w|=reg(t[1])<<21|reg(t[2])<<16|reg(t[3])<<11|(rb&7)<<8|(wm<<4)|((rb>>3)&3)<<2
        elif mn=='TEX':                              # TEX rd, rs0(coord), #samp[, BIAS | LOD]: imm[5:4] = mode
            mode={'BIAS':1,'LOD':2}[t[4].upper()] if len(t)>4 and t[4] else 0   # 1: bias = rs0.w, 2: LOD = rs0.w
            w|=reg(t[1])<<21|reg(t[2])<<16|(int(str(t[3]).lstrip('#'))&0xf)|(mode<<4)
        elif mn=='STOZ':                             # gl_FragDepth: STOZ rs0
            w|=reg(t[1])<<16
        elif mn=='SETP':
            w|=reg(t[2])<<16|reg(t[3])<<11|CMP[t[1].upper()]<<4   # SETP SLT, rs0, rs1
        elif mn=='SET':
            w|=reg(t[2])<<21|reg(t[3])<<16|reg(t[4])<<11|CMP[t[1].upper()]<<4   # SET SLT, rd, rs0, rs1
        elif mn in ('LDP','LDV'):
            w|=reg(t[1])<<21|(int(t[2])&0xffff)
        elif mn in ('STV','STO'):
            w|=reg(t[2])<<16|(int(t[1])&0xffff)                  # STO outidx, rs0
        elif mn in ('BRA','BRP','LOOP','IF','ELSE'):
            tgt=labels[t[1]] if t[1] in labels else int(t[1]); w|=tgt&0xffff
            if mn=='BRP' and len(t)>2 and t[2].upper()=='F': w|=1<<25  # branch-if-false flag = ins[25]
            #   (the unused rd field); NOT ins[2], which overlapped imm[2] and corrupted the branch target.
        elif mn in ('NOP','ENDL','ENDIF','BREAK','CONT','DISCARD','END'): pass
        else: raise ValueError("bad op "+mn)
        words.append(w); src.append(ln)
    return words, labels

# ---- vec4 helpers over FMT bits (4 lanes packed little-endian) ----
# Core routes ADD/SUB/MUL through the cluster's madd (a*b +/- c), so the
# reference mirrors that exactly: ONE/ZERO constants, madd composition.
ONE  = fr.encode_real(F(1), FMT)
ZERO = 0
def neg(x): return x ^ (1<<(FMT.W-1))
def madd1(a,b,c,sub):  # fadd(fmul(a,b), +/-c)  -- matches float_madd
    return fr.fadd(fr.fmul(a,b,FMT), neg(c) if sub else c, FMT)
def v_splat(x): return [x,x,x,x]
def vmul(a,b): return [madd1(a[i],b[i],ZERO,0) for i in range(4)]    # a*b+0
def vadd(a,b): return [madd1(a[i],ONE,b[i],0) for i in range(4)]     # a*1+b
def vsub(a,b): return [madd1(a[i],ONE,b[i],1) for i in range(4)]     # a*1-b
def vdot(a,b,n):
    p=[fr.fmul(a[i],b[i],FMT) for i in range(n)]
    s=p[0]
    for i in range(1,n): s=fr.fadd(s,p[i],FMT)
    # match RTL tree (p0+p1)+(p2+p3) for n=4
    if n==4: s=fr.fadd(fr.fadd(p[0],p[1],FMT),fr.fadd(p[2],p[3],FMT),FMT)
    elif n==3: s=fr.fadd(fr.fadd(p[0],p[1],FMT),p[2],FMT)
    return v_splat(s)

def applymod_f(v, m, FMT):   # per-lane source modifier on FMT bits: 1 neg, 2 abs, 3 -abs
    if m==0: return v[:]
    sb = 1 << (FMT.W-1)
    return [(x ^ sb) if m==1 else (x & ~sb) if m==2 else (x | sb) for x in v]
def applymod(v, m): return applymod_f(v, m, FMT)

# ---- integer ALU helpers: registers hold 2's-comp ints in the W-bit lane ----
def _isigned(bits, W):
    b = bits & ((1<<W)-1)
    return b-(1<<W) if (b>>(W-1)) else b
def _iwrap(v, W): return v & ((1<<W)-1)
def int_op(a, b, code, FMT, n):
    W=FMT.W; one=fr.encode_real(F(1),FMT)
    out=[]
    for i in range(n):
        sa=_isigned(a[i],W); sb=_isigned(b[i],W)
        if   code==0: out.append(_iwrap(sa+sb,W))
        elif code==1: out.append(_iwrap(sa-sb,W))
        elif code==2: out.append(one if sa< sb else 0)
        elif code==3: out.append(one if sa>=sb else 0)
        elif code==4: out.append(one if sa==sb else 0)
        elif code==5: out.append(one if sa!=sb else 0)
        elif code==6: out.append(_iwrap(sa*sb,W))                       # IMUL: low W bits
        elif code==7:                                                   # IDIV: toward zero; x/0 = -1
            if sb==0: out.append(_iwrap(-1,W))
            else:
                q=abs(sa)//abs(sb); out.append(_iwrap(-q if (sa<0)!=(sb<0) else q, W))   # -2^(W-1)/-1 wraps
        elif code==8:                                                   # IMOD: sign of the dividend; x%0 = x
            r=abs(sa)%abs(sb) if sb!=0 else abs(sa); out.append(_iwrap(-r if sa<0 else r, W))
        elif code in (9,10):                                            # UDIV / UMOD; x/0 = all ones, x%0 = x
            ua=a[i]&((1<<W)-1); ub=b[i]&((1<<W)-1)
            if code==9: out.append(ua//ub if ub else (1<<W)-1)
            else:       out.append(ua%ub if ub else ua)
        else:                                                           # IOP: code 16 + fn
            fn=code-16; ua=a[i]&((1<<W)-1); ub=b[i]&((1<<W)-1); n=ub&31
            if   fn==0:  r=ua&ub
            elif fn==1:  r=ua|ub
            elif fn==2:  r=ua^ub
            elif fn==3:  r=~ua
            elif fn==4:  r=(ua<<n) if n<W else 0
            elif fn==5:  r=(sa>>n) if n<W else (-1 if sa<0 else 0)
            elif fn==6:  r=(ua>>n) if n<W else 0
            elif fn==7:  r=min(sa,sb)
            elif fn==8:  r=max(sa,sb)
            elif fn==9:  r=min(ua,ub)
            elif fn==10: r=max(ua,ub)
            elif fn==11: r=abs(sa)
            elif fn==12: r=-sa
            elif fn==13: r=-1 if sa<0 else (1 if sa else 0)
            elif fn==14: r=one if ua< ub else 0
            else:        r=one if ua>=ub else 0
            out.append(r if fn in (14,15) else _iwrap(r,W))
    return out
def _intcode(op, w=0):
    if op==OPS['IOP']: return 16 + (w & 0xF)
    return {OPS['IADD']:0, OPS['ISUB']:1, OPS['ISLT']:2, OPS['ISGE']:3, OPS['ISEQ']:4, OPS['ISNE']:5,
            OPS['IMUL']:6, OPS['IDIV']:7, OPS['IMOD']:8, OPS['UDIV']:9, OPS['UMOD']:10}[op]
INT_OPS = ('IADD','ISUB','ISLT','ISGE','ISEQ','ISNE','IMUL','IDIV','IMOD','UDIV','UMOD','IOP')

_ALU_OPS = None
def _is_alu(op):
    global _ALU_OPS
    if _ALU_OPS is None:
        _ALU_OPS = {OPS[k] for k in ('MOV','ADD','SUB','MUL','MIN','MAX','DP3','DP4','RCP','RSQRT','LOG2','EXP2','SIN','COS','FTOI','ITOF','FTOU','UTOF')+RND_OPS}
    return op in _ALU_OPS

def simulate(words, params, varys, max_steps=100000):
    # sin/cos golden = the RTL's interpolated-table model (bit-exact), as in simulate_simt
    _sk=fr.SIN_KSIN[FMT.name]; _sc=fr.SIN_CFRAC[FMT.name]; _sC=fr.sin_C(FMT); _slut=fr.gen_sin_lut(FMT,_sk)
    gpr=[[0,0,0,0] for _ in range(NREG)]
    out_pos=[0,0,0,0]; out_v=[[0,0,0,0] for _ in range(NVARY)]
    pc=0; pred=0; lstack=[]; steps=0
    while steps<max_steps:
        steps+=1
        w=words[pc] if pc<len(words) else (OPS['END']<<26)
        op=(w>>26)&0x3f; rd=(w>>21)&0x1f; rs0=(w>>16)&0x1f; rs1=(w>>11)&0x1f
        rs2=(w>>8)&7; sub=(w>>4)&0xf; pf=w&0xf; imm=w&0xffff
        predok = (not (pf&8)) or (pred == (0 if (pf&4) else 1))
        nxt=pc+1
        # source modifiers on arithmetic ALU ops: ins[9:8]=src0, ins[1:0]=src1
        _alu = _is_alu(op)
        a=applymod(gpr[rs0], ((w>>8)&3) if _alu else 0)
        b=applymod(gpr[rs1], (w&3) if (_alu or op==OPS['MAD']) else 0)   # MAD src1 modifier ([9:8]=rs2 blocks src0)
        if   op==OPS['NOP']: pass
        elif op==OPS['MOV']:  res=a[:]
        elif op==OPS['ADD']:  res=vadd(a,b)
        elif op==OPS['SUB']:  res=vsub(a,b)
        elif op==OPS['MUL']:  res=vmul(a,b)
        elif op==OPS['MAD']:  res=[madd1(a[i],b[i],gpr[sel_rb(w)][i],0) for i in range(4)]
        elif op==OPS['MIN']:  res=[a[i] if _sr(a[i],FMT)<_sr(b[i],FMT) else b[i] for i in range(4)]
        elif op==OPS['MAX']:  res=[a[i] if _sr(a[i],FMT)>_sr(b[i],FMT) else b[i] for i in range(4)]
        elif op==OPS['SET']:  res=[fr.encode_real(F(1),FMT) if _setp(a[i],b[i],sub,FMT) else 0 for i in range(4)]
        elif op==OPS['DP3']:  res=vdot(a,b,3)
        elif op==OPS['DP4']:  res=vdot(a,b,4)
        elif op==OPS['RCP']:  res=[_enc_sfu(_frcp(_sr(a[i],FMT)), FMT) for i in range(4)]
        elif op==OPS['RSQRT']:res=[_enc_sfu(_frsqrt(_sr(a[i],FMT)), FMT) for i in range(4)]
        elif op==OPS['LOG2']: res=[_enc_sfu(_flog2(_sr(a[i],FMT)), FMT) for i in range(4)]
        elif op==OPS['EXP2']: res=[_enc_sfu(_fexp2(_sr(a[i],FMT)), FMT) for i in range(4)]
        elif op==OPS['SIN']:  res=[fr.sfu_sin(a[i],FMT,_slut,_sk,_sc,_sC,False) for i in range(4)]
        elif op==OPS['COS']:  res=[fr.sfu_sin(a[i],FMT,_slut,_sk,_sc,_sC,True)  for i in range(4)]
        elif op==OPS['FTOI']: res=[ftoi(a[i],FMT) for i in range(4)]
        elif op==OPS['ITOF']: res=[itof(a[i],FMT) for i in range(4)]
        elif op==OPS['FTOU']: res=[ftou(a[i],FMT) for i in range(4)]
        elif op==OPS['UTOF']: res=[utof(a[i],FMT) for i in range(4)]
        elif OPS['TRUNC']<=op<=OPS['RNDE']: res=[fr.fround(a[i],FMT,op-OPS['TRUNC']) for i in range(4)]
        elif op==OPS['SEL']:  res=[sel_lane(gpr[rs0][i],gpr[rs1][i],gpr[sel_rb(w)][i],FMT) for i in range(4)]
        elif op in {OPS[k] for k in INT_OPS}:
            res=int_op(gpr[rs0],gpr[rs1],_intcode(op,w),FMT,4)
        elif op==OPS['SETP']:
            pred = _setp(a[0],b[0],sub,FMT)                    # CMP sub-op ins[7:4] -> SLT/SGE/SEQ/SNE
            res=None
        elif op==OPS['LDP']: res=params[imm][:]
        elif op==OPS['LDV']: res=varys[imm][:]
        elif op==OPS['STV']: out_v[imm]=a[:]; res=None
        elif op==OPS['STO']: out_pos=a[:]; res=None
        elif op==OPS['BRA']: nxt=imm; res=None
        elif op==OPS['BRP']:
            res=None
            if pred==(0 if ((w>>25)&1) else 1): nxt=imm     # branch-if-false flag = ins[25] (not pf&4)
        elif op==OPS['LOOP']: lstack.append([imm, pc+1]); res=None
        elif op==OPS['ENDL']:
            res=None
            if lstack:
                lstack[-1][0]-=1
                if lstack[-1][0]>0: nxt=lstack[-1][1]
                else: lstack.pop()
        elif op==OPS['END']: break
        else: raise ValueError("sim bad op %d"%op)
        # writeback. NOTE: the ISA has NO per-instruction predication (the RTL gates writeback only by
        # wr_gpr, never by pf). pf/predok would mis-fire for LD ops whose imm[3:0] overlaps the pf field,
        # so writeback is unconditional (control flow uses BRP / SETP+IF, not per-op predication).
        if op in (OPS['MOV'],OPS['ADD'],OPS['SUB'],OPS['MUL'],OPS['MAD'],OPS['MIN'],
                  OPS['MAX'],OPS['SET'],OPS['DP3'],OPS['DP4'],OPS['RCP'],OPS['RSQRT'],OPS['LOG2'],
                  OPS['EXP2'],OPS['SIN'],OPS['COS'],OPS['FTOI'],OPS['ITOF'],OPS['IADD'],OPS['ISUB'],OPS['ISLT'],OPS['ISGE'],
                  OPS['ISEQ'],OPS['ISNE'],OPS['IMUL'],OPS['IDIV'],OPS['IMOD'],OPS['UDIV'],OPS['UMOD'],OPS['IOP'],
                  OPS['FTOU'],OPS['UTOF'],
                  OPS['TRUNC'],OPS['FLR'],OPS['CEIL'],OPS['RNDE'],OPS['SEL'],
                  OPS['LDP'],OPS['LDV']):
            # component write-mask ins[7:4]; LDP/LDV/SET always full (imm/sub-op overlaps the field)
            cm = 0xF if (op in (OPS['LDP'],OPS['LDV'],OPS['SET'])) else sub
            gpr[rd]=[res[i] if (cm&(1<<i)) else gpr[rd][i] for i in range(4)]
        pc=nxt
    return gpr, out_pos, out_v

# ============ SIMT (FS) reference: 4 threads x 4 lanes = 16 lanes ============
def simulate_simt(words, params, varys, FMT, max_steps=200000, want_depth=False, tex_fn=None):
    """params[i] = one vec4 (4 lane bits, broadcast to all threads).
       varys[i]  = 16 lane bits (per-thread). gpr[r] = 16 lane bits.
       lane l = thread(l//4), comp(l%4).
       want_depth: also return gl_FragDepth (STOZ rs0.x per active, not killed thread; None if never written).
       tex_fn(sampler, mode, coords): TEX (coords = rs0's 16 lanes, all 4 pixels; -> 16 lanes RGBA), e.g. tmu_ref."""
    outz=[0]*4; wrote_z=False
    o = fr.encode_real(F(1),FMT); z=0
    def m1(a,b,c,s): return fr.fadd(fr.fmul(a,b,FMT), (c^(1<<(FMT.W-1))) if s else c, FMT)
    # sin/cos golden = the SAME no-lerp quarter-wave LUT model as the RTL (bit-exact)
    _sk=fr.SIN_KSIN[FMT.name]; _sc=fr.SIN_CFRAC[FMT.name]; _sC=fr.sin_C(FMT); _slut=fr.gen_sin_lut(FMT,_sk)
    gpr=[[0]*16 for _ in range(NREG)]; outc=[0]*16
    active=0xF; pred=0; mstack=[]; lstack=[]; pc=0; steps=0; killed=0
    WR={OPS[k] for k in ('MOV','ADD','SUB','MUL','MAD','MIN','MAX','SET','DP3','DP4','LDP','LDV','RCP','RSQRT','LOG2','EXP2','SIN','COS','FTOI','ITOF','SEL','FTOU','UTOF')+INT_OPS+RND_OPS}
    while steps<max_steps:
        steps+=1
        w=words[pc] if pc<len(words) else (OPS['END']<<26)
        op=(w>>26)&0x3f; rd=(w>>21)&0x1f; rs0=(w>>16)&0x1f; rs1=(w>>11)&0x1f
        rs2=(w>>8)&7; imm=w&0xffff; sub=(w>>4)&0xf; nxt=pc+1; res=None
        _al=_is_alu(op)
        a=applymod_f(gpr[rs0], ((w>>8)&3) if _al else 0, FMT)   # src0 modifier ins[9:8]
        b=applymod_f(gpr[rs1], (w&3) if (_al or op==OPS['MAD']) else 0, FMT)   # src1 mod (MAD too; [9:8]=rs2)
        if   op==OPS['MUL']: res=[m1(a[l],b[l],z,0) for l in range(16)]
        elif op==OPS['ADD']: res=[m1(a[l],o,b[l],0) for l in range(16)]
        elif op==OPS['SUB']: res=[m1(a[l],o,b[l],1) for l in range(16)]
        elif op==OPS['MAD']: res=[m1(a[l],b[l],gpr[sel_rb(w)][l],0) for l in range(16)]
        elif op==OPS['MOV']: res=a[:]
        elif op==OPS['MIN']: res=[a[l] if _sr(a[l],FMT)<_sr(b[l],FMT) else b[l] for l in range(16)]
        elif op==OPS['MAX']: res=[a[l] if _sr(a[l],FMT)>_sr(b[l],FMT) else b[l] for l in range(16)]
        elif op==OPS['SET']: res=[o if _setp(a[l],b[l],sub,FMT) else 0 for l in range(16)]   # exact 1.0/0.0
        elif op==OPS['RCP']:   res=[_enc_sfu(_frcp(_sr(a[l],FMT)), FMT) for l in range(16)]
        elif op==OPS['RSQRT']: res=[_enc_sfu(_frsqrt(_sr(a[l],FMT)), FMT) for l in range(16)]
        elif op==OPS['LOG2']:  res=[_enc_sfu(_flog2(_sr(a[l],FMT)), FMT) for l in range(16)]
        elif op==OPS['EXP2']:  res=[_enc_sfu(_fexp2(_sr(a[l],FMT)), FMT) for l in range(16)]
        elif op==OPS['SIN']:   res=[fr.sfu_sin(a[l],FMT,_slut,_sk,_sc,_sC,False) for l in range(16)]
        elif op==OPS['COS']:   res=[fr.sfu_sin(a[l],FMT,_slut,_sk,_sc,_sC,True)  for l in range(16)]
        elif op==OPS['FTOI']:  res=[ftoi(a[l],FMT) for l in range(16)]
        elif op==OPS['ITOF']:  res=[itof(a[l],FMT) for l in range(16)]
        elif op==OPS['FTOU']:  res=[ftou(a[l],FMT) for l in range(16)]
        elif op==OPS['UTOF']:  res=[utof(a[l],FMT) for l in range(16)]
        elif OPS['TRUNC']<=op<=OPS['RNDE']: res=[fr.fround(a[l],FMT,op-OPS['TRUNC']) for l in range(16)]
        elif op==OPS['SEL']:   res=[sel_lane(gpr[rs0][l],gpr[rs1][l],gpr[sel_rb(w)][l],FMT) for l in range(16)]
        elif op in {OPS[k] for k in INT_OPS}:
            res=int_op(gpr[rs0],gpr[rs1],_intcode(op,w),FMT,16)
        elif op in (OPS['DP3'],OPS['DP4']):
            n=3 if op==OPS['DP3'] else 4; res=[0]*16
            for p in range(4):
                pr=[fr.fmul(a[p*4+c],b[p*4+c],FMT) for c in range(4)]
                s=fr.fadd(fr.fadd(pr[0],pr[1],FMT),(fr.fadd(pr[2],pr[3],FMT) if n==4 else pr[2]),FMT)
                for c in range(4): res[p*4+c]=s
        elif op==OPS['LDP']: res=[params[imm][l%4] for l in range(16)]      # broadcast vec4
        elif op==OPS['LDV']: res=varys[imm][:]
        elif op==OPS['STO']:
            for p in range(4):
                if (active&(1<<p)) and not ((killed>>p)&1):
                    for c in range(4): outc[p*4+c]=a[p*4+c]
        elif op==OPS['DISCARD']: killed |= active
        elif op==OPS['TEX'] and tex_fn is not None:
            res=tex_fn(imm&0xF, (imm>>4)&3, gpr[rs0][:])        # no source modifiers on TEX
        elif op==OPS['STOZ']:
            wrote_z=True
            for p in range(4):
                if (active&(1<<p)) and not ((killed>>p)&1): outz[p]=gpr[rs0][p*4]
        elif op==OPS['SETP']:
            pred=0
            for p in range(4):
                if _setp(a[p*4],b[p*4],sub,FMT): pred|=(1<<p)   # CMP sub-op ins[7:4] -> SLT/SGE/SEQ/SNE
        elif op==OPS['IF']:
            then=active&pred; mstack.append((active,then)); active=then
            if active==0: nxt=imm
        elif op==OPS['ELSE']:
            ent,then=mstack[-1]; ll=(lstack[-1][3]&~lstack[-1][4]) if lstack else 0xF
            active=ent&(~then)&ll&0xF                            # reconverge, excluding broke/continued lanes
            if active==0: nxt=imm
        elif op==OPS['ENDIF']:
            ent=mstack.pop()[0]; ll=(lstack[-1][3]&~lstack[-1][4]) if lstack else 0xF
            active=ent&ll&0xF
        elif op==OPS['LOOP']: lstack.append([imm,pc+1,active,active,0])  # cnt,body,ent,loopmask,contmask
        elif op==OPS['BREAK']:
            if lstack:
                b=active&pred; lstack[-1][3]&=(~b)&0xF; active&=(~b)&0xF     # leave the loop
        elif op==OPS['CONT']:
            if lstack:
                c=active&pred; lstack[-1][4]|=c&0xF; active&=(~c)&0xF        # skip rest of iteration, rejoin next
        elif op==OPS['ENDL']:
            if lstack:
                lstack[-1][0]-=1
                if lstack[-1][0]>0 and lstack[-1][3]!=0:
                    active=lstack[-1][3]; lstack[-1][4]=0; nxt=lstack[-1][1]  # next iter: continued lanes rejoin
                else:
                    active=lstack[-1][2]; lstack.pop()           # exit: restore entering mask
        elif op==OPS['BRA']: nxt=imm
        elif op==OPS['END']: break
        if res is not None and (op in WR or op==OPS['TEX']):
            cm = 0xF if (op in (OPS['LDP'],OPS['LDV'],OPS['SET'],OPS['TEX'])) else sub   # write-mask ins[7:4]; SET / TEX full
            for p in range(4):
                if active&(1<<p):
                    for c in range(4):
                        if cm&(1<<c): gpr[rd][p*4+c]=res[p*4+c]
        pc=nxt
    if want_depth: return gpr, outc, killed, (outz if wrote_z else None)
    return gpr, outc, killed

def main_fs(prog, FMT=fr.FP16, tag="fs"):
    # tag "fs": FP16 (the single-board FS core); "fs24": FP24, the vertex core's precision (board B's FS core)
    name=os.path.splitext(os.path.basename(prog))[0]
    words,_=assemble(open(prog).read().splitlines())
    def fb(x): return fr.encode_real(F(x).limit_denominator(1<<14),FMT)
    # params: one vec4 each (broadcast). varyings: per-thread (DIFFERENT per thread -> divergence)
    params=[[fb((i%7)*0.25+0.5)]*4 for i in range(NPARAM)]
    # vary[i] thread p comp c: make thread-dependent so IF/ELSE diverges
    varys=[[fb(((i+1)*(p+1)*0.5 + c*0.125) % 3.0 + 0.25) for p in range(4) for c in range(4)] for i in range(NVARY)]
    gpr,outc,killed=simulate_simt(words,params,varys,FMT)
    hw=(FMT.W+3)//4
    def vhex16(v): return "".join(f"{v[15-k]:0{hw}x}" for k in range(16))  # lane15..0
    def vhex4(v):  return "".join(f"{v[3-k]:0{hw}x}" for k in range(4))
    with open(f"{GEN}/{name}_{tag}_outc.mem","w") as f: f.write(vhex16(outc)+"\n")
    with open(f"{GEN}/{name}_{tag}_kill.mem","w") as f: f.write(f"{killed:x}\n")
    with open(f"{GEN}/{name}_{tag}_iram.mem","w") as f:
        for w in words: f.write(f"{w:08x}\n")
    with open(f"{GEN}/{name}_{tag}_params.mem","w") as f:
        for v in params: f.write(vhex4(v)+"\n")
    with open(f"{GEN}/{name}_{tag}_varying.mem","w") as f:
        for v in varys: f.write(vhex16(v)+"\n")
    with open(f"{GEN}/{name}_{tag}_expgpr.mem","w") as f:
        for v in gpr: f.write(vhex16(v)+"\n")
    print(f"{name} (FS/SIMT, {FMT.name}): {len(words)} instr -> tools/gen/{name}_{tag}_*.mem")

def main():
    if '--fs24' in sys.argv:
        main_fs([a for a in sys.argv[1:] if a!='--fs24'][0], fr.FP24, 'fs24'); return
    if '--fs' in sys.argv:
        main_fs([a for a in sys.argv[1:] if a!='--fs'][0]); return
    prog=sys.argv[1]; name=os.path.splitext(os.path.basename(prog))[0]
    words,_=assemble(open(prog).read().splitlines())
    # deterministic test params/varyings: param[i] = (i+1) splatted small floats; vary[i] similar
    def fbits(x): return fr.encode_real(F(x).limit_denominator(1<<20),FMT)
    params=[v_splat(fbits((i%7)*0.25+0.5)) for i in range(NPARAM)]
    varys =[v_splat(fbits((i%5)*0.5+0.25)) for i in range(NVARY)]
    gpr,pos,outv=simulate(words,params,varys)
    hw=(FMT.W+3)//4; vw=4*FMT.W//4
    def vhex(v): return "".join(f"{v[3-k]:0{hw}x}" for k in range(4))  # lane3..0 = MSB..LSB
    with open(f"{GEN}/{name}_iram.mem","w") as f:
        for w in words: f.write(f"{w:08x}\n")
    with open(f"{GEN}/{name}_params.mem","w") as f:
        for v in params: f.write(vhex(v)+"\n")
    with open(f"{GEN}/{name}_varying.mem","w") as f:
        for v in varys: f.write(vhex(v)+"\n")
    with open(f"{GEN}/{name}_expgpr.mem","w") as f:
        for v in gpr: f.write(vhex(v)+"\n")
    print(f"{name}: {len(words)} instr -> tools/gen/{name}_(iram|params|varying|expgpr).mem")
    print(f"  out_pos = {vhex(pos)}")

if __name__=="__main__": main()
