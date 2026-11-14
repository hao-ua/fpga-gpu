#!/usr/bin/env python3
"""gpu2 float reference model + SFU validation + artifact generator.

Formats (sign, exp, man):  FP16=(1,5,10)  FP24=(1,7,16).  FTZ, round-to-nearest-even.

Run:  python3 tools/float_ref.py
It (1) self-tests the codec, (2) golden mul/add via exact rationals,
(3) emulates the SFU (LUT seed + Newton / LUT+interp) and reports max ULP,
(4) emits seed .mem tables and golden test-vector files under tools/gen/.
"""
import os, struct, random
from fractions import Fraction as F

random.seed(1)
GEN = os.path.join(os.path.dirname(__file__), "gen")
os.makedirs(GEN, exist_ok=True)

class Fmt:
    def __init__(self, exp_w, man_w, name):
        self.E, self.M, self.name = exp_w, man_w, name
        self.W = 1 + exp_w + man_w
        self.bias = (1 << (exp_w-1)) - 1
        self.emax = self.bias              # exp field all-ones is Inf/NaN
        self.emin = 1 - self.bias          # smallest normal exponent
        self.inf_exp = (1 << exp_w) - 1
FP16 = Fmt(5, 10, "fp16")
FP24 = Fmt(7, 16, "fp24")

# ---------- decode: bits -> ('num', Fraction) | ('inf',sign) | ('nan',) | ('zero',sign)
def decode(bits, f):
    s  = (bits >> (f.E + f.M)) & 1
    e  = (bits >> f.M) & ((1 << f.E) - 1)
    m  =  bits & ((1 << f.M) - 1)
    if e == f.inf_exp:
        return ('nan',) if m else ('inf', s)
    if e == 0:                              # FTZ: treat subnormals as signed zero
        return ('zero', s)
    val = F(( (1 << f.M) | m ), 1) * F(2) ** (e - f.bias - f.M)
    return ('num', -val if s else val, s)

# ---------- encode: real value (Fraction) -> bits, RNE + FTZ + overflow ----------
def encode_real(v, f):
    if v == 0:
        return 0
    s = 1 if v < 0 else 0
    a = -v if v < 0 else v
    # find e with significand in [1,2): a = 2^e * (1.frac)
    e = 0
    two = F(2)
    while a >= two: a /= two; e += 1
    while a < 1:    a *= two; e -= 1
    if e < f.emin:                          # underflow -> FTZ signed zero
        return s << (f.E + f.M)
    # scale mantissa to integer in [2^M, 2^{M+1}) and round-to-nearest-even
    scaled = a * (1 << f.M)                 # in [2^M, 2^{M+1})
    fl = int(scaled)                        # floor
    frac = scaled - fl
    if frac > F(1, 2) or (frac == F(1, 2) and (fl & 1)):
        fl += 1
    if fl == (1 << (f.M + 1)):              # rounded up to next binade
        fl >>= 1; e += 1
    if e > f.emax:                          # overflow -> Inf
        return (s << (f.E + f.M)) | (f.inf_exp << f.M)
    man = fl & ((1 << f.M) - 1)
    return (s << (f.E + f.M)) | ((e + f.bias) << f.M) | man

def to_real(bits, f):
    d = decode(bits, f)
    return d[1] if d[0] == 'num' else (F(0) if d[0] == 'zero' else None)

def ulp_err(approx_bits, true_val, f):
    """unit-in-last-place error of approx vs the real true_val."""
    if true_val == 0:
        return 0.0 if to_real(approx_bits, f) == 0 else float('inf')
    nearest = encode_real(true_val, f)
    # ulp size at the true value's exponent
    a = abs(true_val); e = 0
    while a >= 2: a /= 2; e += 1
    while a < 1:  a *= 2; e -= 1
    ulp = F(2) ** (e - f.M)
    av = to_real(approx_bits, f)
    if av is None: return float('inf')
    return float(abs(av - true_val) / ulp)

# ================= golden ALU (exact) =================
def fmul(a, b, f):
    da, db = decode(a, f), decode(b, f)
    for d, other in ((da, db), (db, da)):
        if d[0] == 'nan': return qnan(f)
    if 'inf' in (da[0], db[0]):
        if 'zero' in (da[0], db[0]): return qnan(f)
        s = da[-1] ^ db[-1]; return inf(s, f)
    if 'zero' in (da[0], db[0]):
        return zero((da[-1] ^ db[-1]), f)
    return encode_real(da[1] * db[1], f)

def fadd(a, b, f):
    da, db = decode(a, f), decode(b, f)
    if da[0] == 'nan' or db[0] == 'nan': return qnan(f)
    if da[0] == 'inf' or db[0] == 'inf':
        if da[0] == 'inf' and db[0] == 'inf' and da[1] != db[1]: return qnan(f)
        return inf(da[1] if da[0]=='inf' else db[1], f)
    va = da[1] if da[0]=='num' else F(0)
    vb = db[1] if db[0]=='num' else F(0)
    r = va + vb
    if r == 0: return zero(0, f)
    return encode_real(r, f)

def qnan(f): return (f.inf_exp << f.M) | (1 << (f.M-1))
def inf(s, f): return (s << (f.E+f.M)) | (f.inf_exp << f.M)
def zero(s, f): return (s << (f.E+f.M))

# ================= SFU emulation (matches intended RTL) =================
# Reduce x = 2^e * m, m in [1,2) (m as integer significand SIG = 1.M, M+1 bits).

def split(bits, f):
    s = (bits >> (f.E+f.M)) & 1
    e = ((bits >> f.M) & ((1<<f.E)-1)) - f.bias
    sig = (1 << f.M) | (bits & ((1<<f.M)-1))     # integer in [2^M, 2^{M+1})
    return s, e, sig

def gen_rcp_seed(f, kbits):
    """seed[k] ~ 1/m for m in [1,2): index = top kbits of mantissa."""
    N = 1 << kbits; tbl = []
    SW = f.M + 2                                  # seed fractional width
    for i in range(N):
        m = 1 + (i + 0.5)/N                        # bin midpoint
        y = 1.0/m                                  # in (0.5,1]
        tbl.append(int(round(y * (1 << SW))) & ((1<<(SW))-1))
    return tbl, SW

def sfu_rcp(bits, f, seed, SW, kbits):
    s, e, sig = split(bits, f)
    d = decode(bits, f)
    if d[0]=='zero': return inf(s, f)
    if d[0]=='inf':  return zero(s, f)
    if d[0]=='nan':  return qnan(f)
    m = F(sig, 1<<f.M)                              # in [1,2)
    idx = (bits >> (f.M - kbits)) & ((1<<kbits)-1)
    y = F(seed[idx], 1<<SW)                         # ~1/m
    y = y * (2 - m*y)                               # 1 Newton step
    val = y * (F(2)**(-e))
    return encode_real(-val if s else val, f)

def gen_rsqrt_seed(f, kbits):
    """index = {e_lsb, top (kbits-1) mantissa bits}; seed ~ 1/sqrt(reduced)."""
    N = 1 << kbits; tbl=[]; SW=f.M+2
    for i in range(N):
        elsb = i >> (kbits-1)
        mi = i & ((1<<(kbits-1))-1); Nm = 1<<(kbits-1)
        m = 1 + (mi+0.5)/Nm                         # [1,2)
        base = m if elsb==0 else 2*m                # even e: m ; odd e: 2m in [2,4)
        y = 1.0/(base**0.5)
        tbl.append(int(round(y*(1<<SW))) & ((1<<SW)-1))
    return tbl, SW

def sfu_rsqrt(bits, f, seed, SW, kbits):
    s, e, sig = split(bits, f)
    d = decode(bits, f)
    if s and d[0] not in ('zero',): return qnan(f)
    if d[0]=='zero': return inf(0, f)
    if d[0]=='inf':  return zero(0, f)
    if d[0]=='nan':  return qnan(f)
    m = F(sig, 1<<f.M)
    elsb = e & 1
    base = m if elsb==0 else 2*m                    # reduce so result exp is e//2
    er = (e - elsb)//2
    km = kbits-1
    idx = (elsb<<km) | ((bits >> (f.M-km)) & ((1<<km)-1))
    y = F(seed[idx], 1<<SW)
    y = y * (F(3,2) - (base*y*y)/2)                 # Newton: y(1.5 - 0.5 base y^2)
    val = y * (F(2)**(-er))
    return encode_real(val, f)

def gen_unit_lut(func, kbits):
    """LUT of func(t) for t in [0,1) at 2^kbits points + next point for lerp."""
    N = 1<<kbits; FW = 24
    tbl=[int(round(func(i/N)*(1<<FW))) for i in range(N+1)]
    return tbl, FW

# LOG2 near 1.0: |x - 1| < 2^-LOG2_NEAR_K is computed as log2(1+u) = u*H, H = c1 + u*(-c1/2 + u*(c1/3 - u*c1/4))
# (c1 = 1/ln2; series truncation u^4/5 < 2^-22 relative) with u = x - 1 exact: U = u * 2^(M+1), |U| < 2^(M+1-K).
# H in HF-bit fixed point, each Horner product truncated by an arithmetic shift (floor) — bit-exact to float_sfu.
# Elsewhere the table result's absolute accuracy (2^-24) is already relative-accurate (|log2 x| >= 2^-4.6).
LOG2_NEAR_K = 5
LOG2_HF = 23                  # H fraction bits: 23 keeps each Horner product within one DSP48 (13 x 25 bits)
LOG2E = F("1.4426950408889634073599246810018921374266459541529859")   # 1/ln2, 52 digits
def log2_near_consts():
    """K1..K4 = round(c1 * 2^HF / d), d = 1, 2, 3, 4 (float_sfu hard-codes the same integers)."""
    k = lambda d: round(LOG2E * (1 << LOG2_HF) / d)
    return k(1), k(2), k(3), k(4)

def log2_near(bits, f):
    """None if x is not within 2^-LOG2_NEAR_K of 1, else the bit-exact near-1 result (sign included)."""
    s,e,sig = split(bits, f); M = f.M
    if s or e not in (0, -1): return None
    U = 2*(sig - (1 << M)) if e == 0 else sig - (1 << (M+1))      # u * 2^(M+1)
    if abs(U) >= (1 << (M+1-LOG2_NEAR_K)): return None
    K1, K2, K3, K4 = log2_near_consts()
    H1 = K3 - ((U*K4) >> (M+1))
    H2 = -K2 + ((U*H1) >> (M+1))
    H3 = K1 + ((U*H2) >> (M+1))
    R = U*H3                                                        # log2(x) * 2^(M+1+HF)
    if R == 0: return 0
    sg = 1 if R < 0 else 0; V = -R if sg else R
    n = V.bit_length(); ex = n - 1 - (M + 1 + LOG2_HF)
    if n > M + 1:
        sh = n - (M + 1); q = V >> sh; rem = V & ((1<<sh)-1); half = 1 << (sh-1)
        if rem > half or (rem == half and (q & 1)): q += 1
        if q == (1 << (M+1)): q >>= 1; ex += 1
    else:
        q = V << (M + 1 - n)
    be = ex + f.bias
    if be <= 0: return sg << (f.E+f.M)
    return (sg << (f.E+f.M)) | (be << M) | (q & ((1<<M)-1))

def sfu_log2(bits, f, lut, FW, kbits):
    s,e,sig = split(bits, f); d=decode(bits,f)
    if d[0]=='zero': return inf(1,f)
    if s or d[0]=='nan': return qnan(f)
    if d[0]=='inf': return inf(0,f)
    nr = log2_near(bits, f)
    if nr is not None: return nr
    fracbits = bits & ((1<<f.M)-1)
    hi = fracbits >> (f.M-kbits); lo = fracbits & ((1<<(f.M-kbits))-1)
    a = F(lut[hi],1<<FW); b=F(lut[hi+1],1<<FW)
    t = F(lo, 1<<(f.M-kbits))
    logm = a + (b-a)*t                              # log2(m)-... interp, in [0,1)
    val = F(e) + logm
    return encode_real(val, f)

def sfu_exp2(bits, f, lut, FW, kbits):
    d=decode(bits,f)
    if d[0]=='nan': return qnan(f)
    if d[0]=='zero': return encode_real(F(1),f)
    if d[0]=='inf': return zero(0,f) if d[1] else inf(0,f)      # 2^-inf = 0, 2^+inf = +inf (float_sfu: |x| >= 2^(E+3))
    x = d[1]
    import math
    i = math.floor(float(x)); fr = x - i
    if i > f.emax: return inf(0,f)
    if i < f.emin-2: return zero(0,f)
    # 2^fr in [1,2): LUT+interp on fr in [0,1)
    frf = float(fr); idx = int(frf*(1<<kbits)); idx=min(idx,(1<<kbits)-1)
    lo = frf*(1<<kbits)-idx
    a=lut[idx]/(1<<FW); b=lut[idx+1]/(1<<FW)        # a,b = 2^f - 1
    val = (1.0 + a + (b-a)*lo) * (2.0**i)
    return encode_real(F(val), f)

# ---- SIN / COS: interpolated quadrant-folded quarter-wave table (matches sincos_sfu RTL bit for bit) ----
# Fixed-point range reduction:  turns = |x|/(2*pi) ~ (sig*C) * 2^{e-M-CFRAC}.
# Keep PF = 2+KSIN+IFB fraction bits of turns (mod 1): top 2 = quadrant, next KSIN = table index, low IFB =
# interpolation fraction. cos = sin(phase + 0.25 turn). Odd quadrants mirror the phase within the quarter
# (exact negate; a zero in-quarter phase there is the peak, 1.0). The table holds sin((pi/2) i/N) in TFB-bit
# fixed point at the grid points; the value is lut[i] + round((lut[i+1]-lut[i]) * frac / 2^IFB) (lut[N] = 1.0),
# then rounded once (RNE) to the format. sin(-x) = -sin(x) via the input sign; cos is even.
# Small angles: sin(x) = x for |x| < 2^(SIN_SMALL_E+1) (relative error x^2/6 below half an ULP); cos(x) goes
# through the table (absolute accuracy near 1.0 is what matters there).
# Accuracy (FP24): interpolation error <= h^2/8 relative (h = pi/2/1024: 2^-21.7), phase LSB 2^-32 turn, table
# LSB 2^-30 -> well under 1 ULP over |x| <= 2^16; absolute error near the zero crossings ~2^-27.
import math as _math
INV2PI = 1.0 / (2.0*_math.pi)
SIN_KSIN  = {"fp16": 10, "fp24": 10}    # table index bits (table depth N = 2^KSIN)
SIN_IFB   = {"fp16": 20, "fp24": 20}    # interpolation fraction bits
SIN_TFB   = {"fp16": 30, "fp24": 30}    # table fixed-point fraction bits (entries < 2^TFB; 1.0 = 2^TFB)
SIN_CFRAC = {"fp16": 40, "fp24": 48}    # fractional bits of the 1/2pi constant
def SIN_SMALL_E(f): return -((f.M + 2) // 2)  # sin(x) = x when the unbiased exponent e <= this (FP24 -9, FP16 -6)
def sin_C(f):     return round(INV2PI * (1 << SIN_CFRAC[f.name]))   # fixed-point 1/(2pi)

def gen_sin_lut(f, ksin):
    """quarter-wave sine table at the grid points: lut[i] = round(sin((pi/2)*i/N) * 2^TFB), i = 0..N-1
    (N = 2^ksin; lut[N] = 2^TFB is implied). Non-negative; the RTL applies the quadrant sign."""
    N = 1 << ksin; T = SIN_TFB[f.name]
    return [round(_math.sin(_math.pi/2*i/N) * (1 << T)) for i in range(N)]

def sfu_sin(bits, f, lut, ksin, cfrac, C, cos):
    d = decode(bits, f)
    if d[0] in ('nan','inf'): return qnan(f)
    if d[0]=='zero':          return encode_real(F(1),f) if cos else 0   # cos(0)=1, sin(0)=0
    s,e,sig = split(bits, f)                       # e unbiased, sig in [2^M,2^{M+1})
    if not cos and e <= SIN_SMALL_E(f): return bits # sin(x) = x
    M=f.M; IFB=SIN_IFB[f.name]; TFB=SIN_TFB[f.name]; N=1<<ksin
    PF = 2 + ksin + IFB; QB = PF - 2               # QB = in-quarter phase bits
    Pr = sig * C
    RS = M + cfrac - PF - e                         # right-shift to land PF turn-fraction bits
    T  = (Pr >> RS) if RS>=0 else (Pr << (-RS))
    phase = T & ((1<<PF)-1)                          # frac(turns) in PF bits
    if cos: phase = (phase + (1 << QB)) & ((1<<PF)-1)   # +0.25 turn
    Q = phase >> QB                                 # top 2 bits = quadrant
    p = phase & ((1<<QB)-1)
    if Q & 1: p = (1 << QB) - p                     # mirror on odd quadrant (p == 2^QB: the peak)
    if p == (1 << QB):
        V = 1 << TFB
    else:
        i = p >> IFB; fr = p & ((1<<IFB)-1)
        La = lut[i]; Lb = lut[i+1] if i+1 < N else (1 << TFB)
        V = La + (((Lb - La) * fr + (1 << (IFB-1))) >> IFB)
    sign = (Q >> 1) & 1                             # negate in the upper half turn
    if not cos: sign ^= s                           # sin is odd
    if V == 0: return 0
    n = V.bit_length(); ex = n - 1 - TFB            # V * 2^-TFB = 1.xxx * 2^ex
    if n > M + 1:                                   # round to M+1 significant bits, RNE
        sh = n - (M + 1); q = V >> sh; rem = V & ((1<<sh)-1); half = 1 << (sh-1)
        if rem > half or (rem == half and (q & 1)): q += 1
        if q == (1 << (M+1)): q >>= 1; ex += 1
    else:
        q = V << (M + 1 - n)
    be = ex + f.bias
    if be <= 0: return 0                            # FTZ (FP16 only: V >= 2^-30)
    return (sign << (f.E+f.M)) | (be << M) | (q & ((1<<M)-1))

# ================= run: self-test + accuracy + emit =================
def randbits(f):
    while True:
        b = random.getrandbits(f.W)
        e = (b>>f.M)&((1<<f.E)-1)
        if e!=0 and e!=f.inf_exp: return b   # finite normals only

def report(f):
    print(f"\n=== {f.name}  (W={f.W}, bias={f.bias}) ===")
    # codec round-trip
    bad=0
    for _ in range(5000):
        b=randbits(f)
        if encode_real(to_real(b,f),f)!=b: bad+=1
    print(f"  codec round-trip mismatches: {bad}", flush=True)
    # mul/add already exact by construction (Fraction) -> used as golden.
    K = 8 if f is FP16 else 12
    rseed,RSW = gen_rcp_seed(f,K)
    rsseed,RSSW = gen_rsqrt_seed(f, K)
    Klog = 8 if f is FP16 else 11
    log_lut,LFW = gen_unit_lut(lambda t: (lambda m: __import__('math').log2(m))(1+t), Klog)
    exp_lut,EFW = gen_unit_lut(lambda t: 2.0**t - 1.0, Klog)   # store 2^f-1 in [0,1) (fits FW bits)
    def finite_normal(tv):                    # true result lands as a finite normal?
        bb = encode_real(F(tv), f)
        e = (bb >> f.M) & ((1<<f.E)-1)
        return e != 0 and e != f.inf_exp
    def maxulp(fn, truef, n=6000, pos=False):
        mx=0.0
        for _ in range(n):
            b=randbits(f)
            if pos: b &= ~(1<<(f.E+f.M))      # clear sign
            tv=truef(to_real(b,f))
            if tv is None or not finite_normal(tv): continue   # skip FTZ/overflow
            mx=max(mx, ulp_err(fn(b), F(tv), f))
        return mx
    def maxabs(fn, truef, n=6000, pos=False): # absolute error (for log2 near 1)
        mx=0.0
        for _ in range(n):
            b=randbits(f)
            if pos: b &= ~(1<<(f.E+f.M))
            tv=truef(to_real(b,f))
            if tv is None: continue
            av=to_real(fn(b),f)
            if av is None: continue
            mx=max(mx, abs(float(av)-float(tv)))
        return mx
    import math
    u_rcp  = maxulp(lambda b: sfu_rcp(b,f,rseed,RSW,K),   lambda x: 1/x if x!=0 else None)
    u_rsq  = maxulp(lambda b: sfu_rsqrt(b,f,rsseed,RSSW,K),lambda x: 1/math.sqrt(float(x)) if x>0 else None, pos=True)
    a_log  = maxabs(lambda b: sfu_log2(b,f,log_lut,LFW,Klog), lambda x: math.log2(float(x)) if x>0 else None, pos=True)
    def true_exp2(x):
        xf=float(x)
        if xf>=f.emax or xf<=f.emin: return None     # result Inf/0 -> skip ULP
        return 2.0**xf
    u_exp  = maxulp(lambda b: sfu_exp2(b,f,exp_lut,EFW,Klog), true_exp2)
    print(f"  RCP   seed={K}b +1NR : max {u_rcp:.2f} ULP")
    print(f"  RSQRT seed={K}b +1NR : max {u_rsq:.2f} ULP")
    print(f"  LOG2  LUT2^{Klog}+lerp: max {a_log:.2e} abs  (~{a_log*(1<<f.M):.2f} ULP@1.0)")
    print(f"  EXP2  LUT2^{Klog}+lerp: max {u_exp:.2f} ULP")
    # SIN/COS: interpolated quarter-wave table. Absolute error over a wide angle range, ULP error where the true
    # result is >= 1/8 in magnitude (away from the zero crossings, where the input's own rounding dominates), and
    # ULP error of sin over small angles (relative accuracy near 0).
    Ksin=SIN_KSIN[f.name]; Csin=SIN_CFRAC[f.name]; Cval=sin_C(f); sin_lut=gen_sin_lut(f,Ksin)
    def sin_err(cos, lo, hi, n=6000, logscale=False):
        mxa=0.0; mxu=0.0
        for _ in range(n):
            xr = (2.0**random.uniform(lo,hi))*random.choice((-1,1)) if logscale else random.uniform(lo,hi)
            b=encode_real(F(xr),f); xv=to_real(b,f)
            if xv is None or xv==0: continue
            av=to_real(sfu_sin(b,f,sin_lut,Ksin,Csin,Cval,cos),f)
            if av is None: continue
            tv=math.cos(float(xv)) if cos else math.sin(float(xv))
            mxa=max(mxa, abs(float(av)-tv))
            if abs(tv) >= 0.125: mxu=max(mxu, ulp_err(sfu_sin(b,f,sin_lut,Ksin,Csin,Cval,cos), F(tv), f))
        return mxa, mxu
    for nm, cs in (("SIN", False), ("COS", True)):
        a1,u1 = sin_err(cs, -math.pi, math.pi); a2,u2 = sin_err(cs, -1000.0, 1000.0)
        print(f"  {nm}   LUT2^{Ksin}+lerp: |x|<=pi max {a1:.2e} abs, {u1:.2f} ULP (|r|>=1/8); "
              f"|x|<=1000 max {a2:.2e} abs, {u2:.2f} ULP")
    us=0.0
    for _ in range(4000):
        xr=(2.0**random.uniform(-14,-1))*random.choice((-1,1)); b=encode_real(F(xr),f); xv=to_real(b,f)
        us=max(us, ulp_err(sfu_sin(b,f,sin_lut,Ksin,Csin,Cval,False), F(math.sin(float(xv))), f))
    print(f"  SIN   small angles 2^-14..2^-1: max {us:.2f} ULP")
    # emit seed tables
    def emit(name, tbl, width):
        with open(os.path.join(GEN,f"{f.name}_{name}.mem"),"w") as fh:
            for v in tbl: fh.write(f"{v:0{(width+3)//4}x}\n")
    emit("rcp_seed", rseed, RSW); emit("rsqrt_seed", rsseed, RSSW)
    emit("log2_lut", log_lut, LFW); emit("exp2_lut", exp_lut, EFW)
    emit("sin_lut", sin_lut, SIN_TFB[f.name])
    # emit golden mul/add vectors: a b prod sum
    with open(os.path.join(GEN,f"{f.name}_muladd_vectors.hex"),"w") as fh:
        hw=(f.W+3)//4
        for _ in range(2000):
            a=randbits(f); b=randbits(f)
            fh.write(f"{a:0{hw}x} {b:0{hw}x} {fmul(a,b,f):0{hw}x} {fadd(a,b,f):0{hw}x}\n")
    print(f"  emitted seeds + vectors -> tools/gen/{f.name}_*")

def emit_sfu_vectors(f):
    """per-op golden: input_bits expected_bits, restricted to POSITIVE finite-normal
    outputs so a raw-int |a-b| distance equals ULP distance (TB tolerance check)."""
    import math
    hw=(f.W+3)//4
    ops = {
        'rcp':   lambda x: (1/x) if x>0 else None,
        'rsqrt': lambda x: (1/math.sqrt(float(x))) if x>0 else None,
        'log2':  lambda x: math.log2(float(x)) if x>1 else None,   # >1 -> positive
        'exp2':  lambda x: (2.0**float(x)) if (f.emin<float(x)<f.emax) else None,
    }
    for name,fn in ops.items():
        out=[]
        while len(out)<1500:
            b=randbits(f) & ~(1<<(f.E+f.M))          # positive input
            tv=fn(to_real(b,f))
            if tv is None or tv<=0: continue
            eb=encode_real(F(tv),f)
            e=(eb>>f.M)&((1<<f.E)-1)
            if e==0 or e==f.inf_exp: continue        # finite normal only
            out.append((b,eb))
        with open(os.path.join(GEN,f"{f.name}_{name}_vectors.hex"),"w") as fh:
            for b,eb in out: fh.write(f"{b:0{hw}x} {eb:0{hw}x}\n")
    # LOG2 near 1.0 (the polynomial path, both sides of 1, its edges and x = 1): float_ref's own bit-exact result
    # (tb_float_sfu checks these at 0 ULP)
    one = encode_real(F(1), f); rows = []
    for d in list(range(0, 40)) + [(1 << (f.M-k)) + j for k in (4, 5) for j in (-1, 0, 1)]:
        for b in (one + d, one - d):                    # bit neighbours of 1.0 on both sides (incl. the region edges)
            if decode(b, f)[0] == 'num': rows.append(b)
    import random as R2; R2.seed(11)
    while len(rows) < 1500:
        rows.append(encode_real(F(1) + F(R2.uniform(-1, 1)) * F(1, 1 << R2.randint(5, 16)), f))
    with open(os.path.join(GEN,f"{f.name}_log2near_vectors.hex"),"w") as fh:
        lut, FWl = gen_unit_lut(lambda t: math.log2(1+t), 8 if f is FP16 else 11)
        for b in rows: fh.write(f"{b:0{hw}x} {sfu_log2(b, f, lut, FWl, 8 if f is FP16 else 11):0{hw}x}\n")
    emit_exp2_special(f)
    print(f"  emitted SFU golden -> tools/gen/{f.name}_(rcp|rsqrt|log2|exp2|log2near|exp2spec)_vectors.hex")

def emit_exp2_special(f):
    """EXP2 where the result is a limit, exact (tb_float_sfu checks at 0 ULP): the infinities, zero, a NaN, and every
    magnitude from the format's own overflow up to the largest finite number — float_sfu converts the argument to fixed
    point with a 2^(E+3) integer range, and beyond it the integer part would wrap."""
    hw=(f.W+3)//4
    big = 1 << (f.E + 3)                                     # float_sfu's 2^(EXW-1)
    mags = [F(f.emax + 2), F(4 * f.emax), F(big) - F(1, 2), F(big - 1), F(big), F(big) + F(1, 2), F(big + 76), F(2 * big - 8), F(2 * big),
            F(2 * big + 8), F(3 * big - 1), F(4 * big), F(1 << (f.M + 2)), F(1 << (f.M + f.E + 1)), F(1 << (f.M + f.E + 2)), F(2) ** (f.emax - 1)]
    rows = [(inf(1, f), zero(0, f)), (inf(0, f), inf(0, f)), (zero(0, f), encode_real(F(1), f)), (zero(1, f), encode_real(F(1), f)), (qnan(f), qnan(f)),
            ((1 << (f.E + f.M)) - 1 - (1 << f.M), inf(0, f)), ((1 << f.W) - 1 - (1 << f.M), zero(0, f))]     # the largest finite number, both signs
    for m in mags:
        b = encode_real(m, f)
        if decode(b, f)[0] != 'num': continue
        rows.append((b, inf(0, f)))                          # 2^m, m > emax: +inf
        rows.append((b | (1 << (f.E + f.M)), zero(0, f)))    # 2^-m: 0
    with open(os.path.join(GEN,f"{f.name}_exp2spec_vectors.hex"),"w") as fh:
        for b,eb in rows: fh.write(f"{b:0{hw}x} {eb:0{hw}x}\n")

def emit_sincos_vectors(f):
    """per-op golden for SIN/COS: input_bits expected_bits. sfu_sin is bit-exact to the RTL
    (same table + same integer reduction), so the TB expects an EXACT match. Spread inputs over
    signs, quadrants and magnitudes; include the exact specials (0, pi/2, pi, 2pi)."""
    import random as R, math; R.seed(5)
    hw=(f.W+3)//4; Ksin=SIN_KSIN[f.name]; Csin=SIN_CFRAC[f.name]; Cval=sin_C(f)
    lut=gen_sin_lut(f,Ksin)
    specials=[0.0, math.pi/2, -math.pi/2, math.pi, -math.pi, 2*math.pi, 1.0, -1.0, 0.5, 3.0, 10.0, -12.5,
              2.0**-9, -2.0**-9, 2.0**-8, 2.0**-8*1.5, -3e-3, 1e-5, 1000.0, -12345.0]   # small-angle edge, large
    for cos in (False,True):
        name='cos' if cos else 'sin'
        rows=[]
        for xr in specials:
            b=encode_real(F(xr).limit_denominator(1<<(f.M+6)),f)
            rows.append((b, sfu_sin(b,f,lut,Ksin,Csin,Cval,cos)))
        while len(rows)<1500:
            xr=R.uniform(-40.0,40.0) if len(rows)%4 else (2.0**R.uniform(-12,0))*R.choice((-1,1))
            b=encode_real(F(xr).limit_denominator(1<<(f.M+6)),f)
            d=decode(b,f)
            if d[0]!='num': continue
            rows.append((b, sfu_sin(b,f,lut,Ksin,Csin,Cval,cos)))
        with open(os.path.join(GEN,f"{f.name}_{name}_vectors.hex"),"w") as fh:
            for b,eb in rows: fh.write(f"{b:0{hw}x} {eb:0{hw}x}\n")
    print(f"  emitted SIN/COS golden -> tools/gen/{f.name}_(sin|cos)_vectors.hex")

def madd(a,b,c,f):  return fadd(fmul(a,b,f), c, f)          # 2-rounding (matches float_madd)
def dot4(av,bv,f):                                          # tree: (p0+p1)+(p2+p3)
    p=[fmul(av[i],bv[i],f) for i in range(4)]
    return fadd(fadd(p[0],p[1],f), fadd(p[2],p[3],f), f)

def fround(bits, f, mode):
    """float -> float rounding to an integer (ISA TRUNC / FLR / CEIL / RNDE = mode 0 / 1 / 2 / 3), exact (the result
    is always representable). Inf / NaN pass through; zero / denormal (FTZ) -> zero with the input's sign; a zero
    result keeps the input's sign (ceil(-0.3) = -0). Matches float_round.sv bit for bit."""
    import math
    d = decode(bits, f)
    if d[0] in ('nan', 'inf'): return bits
    s = (bits >> (f.E+f.M)) & 1
    if d[0] == 'zero': return s << (f.E+f.M)
    x = d[1]
    r = (math.trunc(x), math.floor(x), math.ceil(x), round(x))[mode]   # round(Fraction): half to even
    if r == 0: return s << (f.E+f.M)
    return encode_real(F(r), f)

def emit_cluster_vectors(f):
    hw=(f.W+3)//4
    def hx(v): return f"{v:0{hw}x}"
    # round: x -> trunc floor ceil rne. Mostly |x| < 2^(M+2) (the interesting range: fractions, halves, the binade
    # edges), plus anything (huge, tiny, specials).
    import random as R3; R3.seed(13)
    xs = [encode_real(F(v), f) for v in (0.5, -0.5, 1.5, -1.5, 2.5, -2.5, 0.49, -0.51, 0.999, -0.999, 1.0, -1.0,
                                         2.0**f.M - 0.5, -(2.0**f.M - 0.5), 2.0**f.M, 3.0*2.0**(f.M-1) + 0.5, 1e-3, -1e-3)]
    xs += [0, 1 << (f.E+f.M), 1, (1 << (f.E+f.M)) | 5, inf(0, f), inf(1, f), qnan(f)]   # +-0, denormals, +-Inf, NaN
    while len(xs) < 3000:
        if R3.random() < 0.8:
            v = F(R3.randint(-(1 << (f.M+3)), 1 << (f.M+3)), 1 << R3.randint(0, 8))
            if v != 0: xs.append(encode_real(v, f))
        else:
            xs.append(randbits(f))
    with open(os.path.join(GEN,f"{f.name}_round_vectors.hex"),"w") as fh:
        for x in xs: fh.write(f"{hx(x)} " + " ".join(hx(fround(x, f, m)) for m in range(4)) + "\n")
    # madd: a b c -> a*b+c
    with open(os.path.join(GEN,f"{f.name}_madd_vectors.hex"),"w") as fh:
        for _ in range(2000):
            a=randbits(f); b=randbits(f); c=randbits(f)
            fh.write(f"{hx(a)} {hx(b)} {hx(c)} {hx(madd(a,b,c,f))}\n")
    # dot4: a0..a3 b0..b3 -> dot
    with open(os.path.join(GEN,f"{f.name}_dot4_vectors.hex"),"w") as fh:
        for _ in range(2000):
            av=[randbits(f) for _ in range(4)]; bv=[randbits(f) for _ in range(4)]
            fh.write(" ".join(hx(v) for v in av+bv) + f" {hx(dot4(av,bv,f))}\n")
    # cmp: a b -> min max slt(1.0/0.0)
    one=encode_real(F(1),f)
    with open(os.path.join(GEN,f"{f.name}_cmp_vectors.hex"),"w") as fh:
        for _ in range(2000):
            a=randbits(f); b=randbits(f); va=to_real(a,f); vb=to_real(b,f)
            mn = a if va<vb else b; mx = a if va>vb else b
            slt = one if va<vb else 0
            fh.write(f"{hx(a)} {hx(b)} {hx(mn)} {hx(mx)} {hx(slt)}\n")
    print(f"  emitted cluster golden -> tools/gen/{f.name}_(round|madd|dot4|cmp)_vectors.hex")

def emit_primpipe(f, NVEC=8):
    # end-to-end golden: w=1 triangle (exact rcp), cull off; per fragment NVEC*4
    # perspective-correct scalar varyings (= affine since w=1).
    import math
    NVAR=NVEC*4
    hw=(f.W+3)//4; one=encode_real(F(1),f)
    sxs=encode_real(F(16),f); sxo=encode_real(F(16),f); sys=encode_real(F(-16),f); syo=encode_real(F(16),f)
    pos=[(-0.5,-0.5,0.2),(0.6,-0.4,0.1),(0.0,0.7,0.3)]
    P=[(encode_real(F(px).limit_denominator(1<<14),f),encode_real(F(py).limit_denominator(1<<14),f),
        encode_real(F(pz).limit_denominator(1<<14),f),one) for (px,py,pz) in pos]
    def vval(v,s): return (v+1)*0.3 + s*0.05            # distinct positive per (vert,scalar)
    VV=[[encode_real(F(vval(v,s)).limit_denominator(1<<14),f) for s in range(NVAR)] for v in range(3)]
    def trunc(b): return int(math.trunc(float(to_real(b,f))))
    S=[(trunc(fadd(fmul(P[v][0],sxs,f),sxo,f)), trunc(fadd(fmul(P[v][1],sys,f),syo,f))) for v in range(3)]
    (x0,y0),(x1,y1),(x2,y2)=S
    a2=(x1-x0)*(y2-y0)-(x2-x0)*(y1-y0)
    def dot3(a,b):
        p=[fmul(a[i],b[i],f) for i in range(3)]; p3=fmul(0,0,f)
        return fadd(fadd(p[0],p[1],f),fadd(p[2],p3,f),f)
    frags=[]
    for yy in range(32):
        for xx in range(32):
            e0=(y2-y1)*(xx-x1)+(x1-x2)*(yy-y1)
            e1=(y0-y2)*(xx-x2)+(x2-x0)*(yy-y2)
            e2=(y1-y0)*(xx-x0)+(x0-x1)*(yy-y0)
            # ES2 coverage samples the pixel CENTER (8,8)/16: test e + ((8*A+8*B)>>4) per edge.
            # (e0/e1/e2 themselves stay the integer-corner values: the rasterizer emits the corner
            #  edge with each fragment, and the perspective interp uses those.)
            e0c=e0+((8*(y2-y1)+8*(x1-x2))>>4)
            e1c=e1+((8*(y0-y2)+8*(x2-x0))>>4)
            e2c=e2+((8*(y1-y0)+8*(x0-x1))>>4)
            ins=(e0c<=0 and e1c<=0 and e2c<=0) if a2>0 else (e0c>=0 and e1c>=0 and e2c>=0)
            if not ins: continue
            ef=[encode_real(F(e0),f),encode_real(F(e1),f),encode_real(F(e2),f)]
            den=dot3(ef,[one,one,one]); dr=to_real(den,f)
            if dr==0: continue
            attrs=[]
            for s in range(NVAR):
                num=dot3(ef,[VV[0][s],VV[1][s],VV[2][s]])
                attrs.append(encode_real(to_real(num,f)/dr, f))
            frags.append((xx,yy,attrs))
    with open(os.path.join(GEN,f"{f.name}_primpipe_vectors.hex"),"w") as fh:
        fh.write(f"{NVEC}\n")
        fh.write(f"{sxs:0{hw}x} {sxo:0{hw}x} {sys:0{hw}x} {syo:0{hw}x}\n")
        for v in range(3):
            fh.write(" ".join(f"{q:0{hw}x}" for q in P[v]) + " " + " ".join(f"{q:0{hw}x}" for q in VV[v]) + "\n")
        fh.write(f"{len(frags)}\n")
        for (xx,yy,attrs) in frags:
            fh.write(f"{xx} {yy} " + " ".join(f"{a:0{hw}x}" for a in attrs) + "\n")
    print(f"  emitted primpipe golden -> tools/gen/{f.name}_primpipe_vectors.hex ({len(frags)} frags x {NVAR} vary)")

def emit_drawpath(f):
    # end-to-end draw golden: VS broadcasts a vec4 color to varyings; prim_pipe
    # interpolates (w=1, affine); FS reads varying#7 -> color narrowed FP24->FP16.
    hw=(f.W+3)//4; one=encode_real(F(1),f)
    sxs=encode_real(F(16),f); sxo=encode_real(F(16),f); sys=encode_real(F(-16),f); syo=encode_real(F(16),f)
    pos=[(-0.5,-0.5,0.2),(0.6,-0.4,0.1),(0.0,0.7,0.3)]
    colr=[(0.2,0.5,0.8,1.0),(0.7,0.3,0.1,0.9),(0.4,0.6,0.25,0.5)]
    P=[(encode_real(F(px).limit_denominator(1<<14),f),encode_real(F(py).limit_denominator(1<<14),f),
        encode_real(F(pz).limit_denominator(1<<14),f),one) for (px,py,pz) in pos]
    COL=[[encode_real(F(colr[v][c]).limit_denominator(1<<14),f) for c in range(4)] for v in range(3)]
    import math
    def trunc(b): return int(math.trunc(float(to_real(b,f))))
    S=[(trunc(fadd(fmul(P[v][0],sxs,f),sxo,f)), trunc(fadd(fmul(P[v][1],sys,f),syo,f))) for v in range(3)]
    (x0,y0),(x1,y1),(x2,y2)=S
    a2=(x1-x0)*(y2-y0)-(x2-x0)*(y1-y0)
    def dot3(a,b):
        p=[fmul(a[i],b[i],f) for i in range(3)]; p3=fmul(0,0,f)
        return fadd(fadd(p[0],p[1],f),fadd(p[2],p3,f),f)
    frags=[]
    for yy in range(32):
        for xx in range(32):
            e0=(y2-y1)*(xx-x1)+(x1-x2)*(yy-y1); e1=(y0-y2)*(xx-x2)+(x2-x0)*(yy-y2); e2=(y1-y0)*(xx-x0)+(x0-x1)*(yy-y0)
            # ES2 coverage = pixel CENTER (8,8)/16; e0/e1/e2 stay corner values (emitted + interp).
            e0c=e0+((8*(y2-y1)+8*(x1-x2))>>4); e1c=e1+((8*(y0-y2)+8*(x2-x0))>>4); e2c=e2+((8*(y1-y0)+8*(x0-x1))>>4)
            ins=(e0c<=0 and e1c<=0 and e2c<=0) if a2>0 else (e0c>=0 and e1c>=0 and e2c>=0)
            if not ins: continue
            ef=[encode_real(F(e0),f),encode_real(F(e1),f),encode_real(F(e2),f)]
            den=dot3(ef,[one,one,one]); dr=to_real(den,f)
            if dr==0: continue
            cvec=[]
            for c in range(4):
                num=dot3(ef,[COL[0][c],COL[1][c],COL[2][c]])
                v24=encode_real(to_real(num,f)/dr,f); cvec.append(encode_real(to_real(v24,f),FP16))
            frags.append((xx,yy,cvec))
    with open(os.path.join(GEN,"drawpath_vectors.hex"),"w") as fh:
        fh.write(f"{sxs:0{hw}x} {sxo:0{hw}x} {sys:0{hw}x} {syo:0{hw}x}\n")
        for v in range(3):
            fh.write(" ".join(f"{q:0{hw}x}" for q in P[v]) + " " + " ".join(f"{q:0{hw}x}" for q in COL[v]) + "\n")
        fh.write(f"{len(frags)}\n")
        for (xx,yy,cvec) in frags: fh.write(f"{xx} {yy} " + " ".join(f"{q:04x}" for q in cvec) + "\n")
    print(f"  emitted drawpath golden -> tools/gen/drawpath_vectors.hex ({len(frags)} frags)")

def emit_clipnear(f, NVAR=8):
    # near-plane clip (Sutherland-Hodgman, keep w>=WEPS=2^-7); intersection verts
    # lerp ALL attrs: out=a+t*(b-a), t=(WEPS-wa)/(wb-wa) (FP24 ops, ideal rcp).
    NATTR=4+NVAR; hw=(f.W+3)//4
    WEPS=encode_real(F(1)/128,f); weps_r=to_real(WEPS,f)
    def ng(x): return x ^ (1<<(f.W-1))
    def mkv(x,y,z,w,vb):
        a=[encode_real(F(x).limit_denominator(1<<14),f),encode_real(F(y).limit_denominator(1<<14),f),
           encode_real(F(z).limit_denominator(1<<14),f),encode_real(F(w).limit_denominator(1<<14),f)]
        a+=[encode_real(F(vb+0.13*k).limit_denominator(1<<14),f) for k in range(NVAR)]
        return a
    tris=[
        [(0.2,0.1,0.0,1.0,0.5),(0.4,-0.3,0.0,2.0,1.0),(0.1,0.5,0.0,-0.5,1.5)],   # 2 in,1 behind
        [(0.2,0.1,0.0,1.0,0.3),(0.4,-0.3,0.0,-0.5,0.7),(0.1,0.5,0.0,-1.0,1.1)],  # 1 in,2 behind
        [(0.2,0.1,0.0,1.0,0.2),(0.4,-0.3,0.0,2.0,0.6),(0.1,0.5,0.0,1.5,1.0)],    # all in
    ]
    cases=[]
    for tri in tris:
        V=[mkv(*v) for v in tri]
        def inside(v): return to_real(v[3],f) >= weps_r
        poly=[]
        for i in range(3):
            cur=V[i]; nxt=V[(i+1)%3]
            if inside(cur): poly.append(cur)
            if inside(cur)!=inside(nxt):
                wa=cur[3]; wb=nxt[3]
                num=fadd(WEPS,ng(wa),f); den=fadd(wb,ng(wa),f)
                t=fmul(num,encode_real(F(1)/to_real(den,f),f),f)
                nv=[fadd(cur[k],fmul(t,fadd(nxt[k],ng(cur[k]),f),f),f) for k in range(NATTR)]
                poly.append(nv)
        out=[]
        if len(poly)>=3: out.append((poly[0],poly[1],poly[2]))
        if len(poly)==4: out.append((poly[0],poly[2],poly[3]))
        cases.append((V,out))
    with open(os.path.join(GEN,"clipnear_vectors.hex"),"w") as fh:
        fh.write(f"{NVAR}\n{len(cases)}\n")
        for V,out in cases:
            for v in V: fh.write(" ".join(f"{q:0{hw}x}" for q in v)+"\n")
            fh.write(f"{len(out)}\n")
            for t3 in out:
                for v in t3: fh.write(" ".join(f"{q:0{hw}x}" for q in v)+"\n")
    print(f"  emitted clipnear golden -> tools/gen/clipnear_vectors.hex ({len(cases)} tris, NVAR={NVAR})")

def emit_primpipe_clip(f, NVEC=8):
    # near-crossing triangle (2 in front, 1 behind near) -> prim_pipe must clip
    # into 2 triangles, then rasterize. Golden clips (Python S-H) + rasterizes.
    import math
    NVAR=NVEC*4; hw=(f.W+3)//4; one=encode_real(F(1),f)
    WEPS=encode_real(F(1)/128,f); weps_r=to_real(WEPS,f)
    sxs=encode_real(F(16),f); sxo=encode_real(F(16),f); sys=encode_real(F(-16),f); syo=encode_real(F(16),f)
    pos=[(-0.4,-0.4,0.0,1.0),(0.5,-0.2,0.0,1.0),(0.0,0.6,0.0,-0.5)]   # vert2 behind near
    def vval(v,s): return (v+1)*0.3 + s*0.05
    def enc(x): return encode_real(F(x).limit_denominator(1<<14),f)
    Vin=[[enc(pos[v][0]),enc(pos[v][1]),enc(pos[v][2]),enc(pos[v][3])] +
         [enc(vval(v,s)) for s in range(NVAR)] for v in range(3)]
    def ng(x): return x^(1<<(f.W-1))
    def inside(v): return to_real(v[3],f) >= weps_r
    poly=[]
    for i in range(3):
        cur=Vin[i]; nxt=Vin[(i+1)%3]
        if inside(cur): poly.append(cur)
        if inside(cur)!=inside(nxt):
            wa=cur[3]; wb=nxt[3]
            num=fadd(WEPS,ng(wa),f); den=fadd(wb,ng(wa),f)
            t=fmul(num,encode_real(F(1)/to_real(den,f),f),f)
            poly.append([fadd(cur[k],fmul(t,fadd(nxt[k],ng(cur[k]),f),f),f) for k in range(4+NVAR)])
    outtris=[]
    if len(poly)>=3: outtris.append([poly[0],poly[1],poly[2]])
    if len(poly)==4: outtris.append([poly[0],poly[2],poly[3]])
    def trunc(b): return int(math.trunc(float(to_real(b,f))))
    def dot3(a,b):
        p=[fmul(a[i],b[i],f) for i in range(3)]; p3=fmul(0,0,f)
        return fadd(fadd(p[0],p[1],f),fadd(p[2],p3,f),f)
    frags={}
    for tri in outtris:
        S=[]; IW=[]
        for v in tri:
            iw=encode_real(F(1)/to_real(v[3],f),f)
            ndcx=fmul(v[0],iw,f); ndcy=fmul(v[1],iw,f)
            S.append((trunc(fadd(fmul(ndcx,sxs,f),sxo,f)), trunc(fadd(fmul(ndcy,sys,f),syo,f)))); IW.append(iw)
        (x0,y0),(x1,y1),(x2,y2)=S
        a2=(x1-x0)*(y2-y0)-(x2-x0)*(y1-y0)
        P=[[fmul(tri[vv][4+s],IW[vv],f) for s in range(NVAR)] for vv in range(3)]
        for yy in range(32):
          for xx in range(32):
            e0=(y2-y1)*(xx-x1)+(x1-x2)*(yy-y1); e1=(y0-y2)*(xx-x2)+(x2-x0)*(yy-y2); e2=(y1-y0)*(xx-x0)+(x0-x1)*(yy-y0)
            # ES2 coverage = pixel CENTER (8,8)/16; e0/e1/e2 stay corner values (emitted + interp).
            e0c=e0+((8*(y2-y1)+8*(x1-x2))>>4); e1c=e1+((8*(y0-y2)+8*(x2-x0))>>4); e2c=e2+((8*(y1-y0)+8*(x0-x1))>>4)
            ins=(e0c<=0 and e1c<=0 and e2c<=0) if a2>0 else (e0c>=0 and e1c>=0 and e2c>=0)
            if not ins: continue
            ef=[encode_real(F(e0),f),encode_real(F(e1),f),encode_real(F(e2),f)]
            den=dot3(ef,IW); dr=to_real(den,f)
            if dr==0: continue
            frags[(xx,yy)]=[encode_real(to_real(dot3(ef,[P[0][s],P[1][s],P[2][s]]),f)/dr,f) for s in range(NVAR)]
    keys=sorted(frags.keys())
    with open(os.path.join(GEN,f"{f.name}_primpipeclip_vectors.hex"),"w") as fh:
        fh.write(f"{NVEC}\n")
        fh.write(f"{sxs:0{hw}x} {sxo:0{hw}x} {sys:0{hw}x} {syo:0{hw}x}\n")
        for v in range(3): fh.write(" ".join(f"{q:0{hw}x}" for q in Vin[v])+"\n")
        fh.write(f"{len(keys)}\n")
        for (xx,yy) in keys: fh.write(f"{xx} {yy} " + " ".join(f"{a:0{hw}x}" for a in frags[(xx,yy)]) + "\n")
    print(f"  emitted primpipe-clip golden -> {f.name}_primpipeclip_vectors.hex ({len(outtris)} tris, {len(keys)} frags)")

def emit_vtxsetup(f, sxs_r, sxo_r, sys_r, syo_r):
    import random as R, math; R.seed(11)
    hw=(f.W+3)//4
    sxs=encode_real(F(sxs_r),f); sxo=encode_real(F(sxo_r),f)
    sys=encode_real(F(sys_r),f); syo=encode_real(F(syo_r),f)
    out=[]
    while len(out)<1200:
        x=encode_real(F(R.uniform(-1.5,1.5)).limit_denominator(1<<14),f)
        y=encode_real(F(R.uniform(-1.5,1.5)).limit_denominator(1<<14),f)
        z=encode_real(F(R.uniform(-1,1)).limit_denominator(1<<14),f)
        w=encode_real(F(R.uniform(0.5,4.0)).limit_denominator(1<<14),f)
        iw=encode_real(F(1)/to_real(w,f),f)
        ndcx=fmul(x,iw,f); ndcy=fmul(y,iw,f); ndcz=fmul(z,iw,f)
        sxf=fadd(fmul(ndcx,sxs,f),sxo,f); syf=fadd(fmul(ndcy,sys,f),syo,f)
        sx=int(math.trunc(float(to_real(sxf,f)))); sy=int(math.trunc(float(to_real(syf,f))))
        out.append((x,y,z,w,sx,sy,iw,ndcz))
    with open(os.path.join(GEN,f"{f.name}_vtxsetup_vectors.hex"),"w") as fh:
        fh.write(f"{sxs:0{hw}x} {sxo:0{hw}x} {sys:0{hw}x} {syo:0{hw}x}\n")
        for x,y,z,w,sx,sy,iw,zn in out:
            fh.write(f"{x:0{hw}x} {y:0{hw}x} {z:0{hw}x} {w:0{hw}x} {sx} {sy} {iw:0{hw}x} {zn:0{hw}x}\n")
    print(f"  emitted vtxsetup golden -> tools/gen/{f.name}_vtxsetup_vectors.hex")

def emit_interp_vectors(f):
    # perspective-correct interp: attr = dot3(ef,p) / dot3(ef,iw), matching the
    # float_dot4 dp3 tree.  e_i small positive ints; iw_i,p_i positive FP24.
    import random as R; R.seed(7)
    hw=(f.W+3)//4
    def i2f(n): return encode_real(F(n),f)
    def dot3(a,b):
        p=[fmul(a[i],b[i],f) for i in range(3)]; p3=fmul(0,0,f)
        return fadd(fadd(p[0],p[1],f), fadd(p[2],p3,f), f)
    out=[]
    while len(out)<1500:
        e=[R.randint(0,80) for _ in range(3)]
        if sum(e)==0: continue
        ef=[i2f(v) for v in e]
        iw=[encode_real(F(R.uniform(0.2,2.0)).limit_denominator(1<<14),f) for _ in range(3)]
        p =[encode_real(F(R.uniform(0.1,3.0)).limit_denominator(1<<14),f) for _ in range(3)]
        den=dot3(ef,iw); num=dot3(ef,p)
        dr=to_real(den,f)
        if dr is None or dr==0: continue
        attr=encode_real(to_real(num,f)/dr, f)
        ee=(attr>>f.M)&((1<<f.E)-1)
        if ee==0 or ee==f.inf_exp: continue
        out.append((e,iw,p,attr))
    with open(os.path.join(GEN,f"{f.name}_interp_vectors.hex"),"w") as fh:
        for e,iw,p,attr in out:
            fh.write(f"{e[0]:08x} {e[1]:08x} {e[2]:08x} "
                     + " ".join(f"{v:0{hw}x}" for v in iw+p)
                     + f" {attr:0{hw}x}\n")
    print(f"  emitted interp golden -> tools/gen/{f.name}_interp_vectors.hex")

def emit_interp_mv(f, NVEC=8, NFRAG=24):
    # multi-varying interp: shared den=dot3(ef,iw); per vec4 comp attr=dot3(ef,P)/den.
    import random as R; R.seed(11)
    hw=(f.W+3)//4
    def i2f(n): return encode_real(F(n),f)
    def dot3(a,b):
        p=[fmul(a[i],b[i],f) for i in range(3)]; p3=fmul(0,0,f)
        return fadd(fadd(p[0],p[1],f), fadd(p[2],p3,f), f)
    lines=[]; nf=0
    while nf<NFRAG:
        e=[R.randint(0,80) for _ in range(3)]
        if sum(e)==0: continue
        ef=[i2f(v) for v in e]
        iw=[encode_real(F(R.uniform(0.2,2.0)).limit_denominator(1<<14),f) for _ in range(3)]
        den=dot3(ef,iw); dr=to_real(den,f)
        if dr is None or dr==0: continue
        vecs=[]; ok=True
        for v in range(NVEC):
            pv=[[encode_real(F(R.uniform(0.1,3.0)).limit_denominator(1<<14),f) for _ in range(3)] for _ in range(4)]  # [comp][vert]
            attr=[]
            for cc in range(4):
                num=dot3(ef,pv[cc]); a=encode_real(to_real(num,f)/dr,f)
                ee=(a>>f.M)&((1<<f.E)-1)
                if ee==0 or ee==f.inf_exp: ok=False
                attr.append(a)
            vecs.append((pv,attr))
        if not ok: continue
        lines.append(f"{e[0]:08x} {e[1]:08x} {e[2]:08x} {iw[0]:0{hw}x} {iw[1]:0{hw}x} {iw[2]:0{hw}x} {NVEC}")
        for pv,attr in vecs:
            pv0=[pv[cc][0] for cc in range(4)]; pv1=[pv[cc][1] for cc in range(4)]; pv2=[pv[cc][2] for cc in range(4)]
            lines.append(" ".join(f"{x:0{hw}x}" for x in pv0+pv1+pv2+attr))
        nf+=1
    with open(os.path.join(GEN,f"{f.name}_interpmv_vectors.hex"),"w") as fh:
        fh.write(f"{NFRAG}\n")
        for l in lines: fh.write(l+"\n")
    print(f"  emitted interp_mv golden -> {f.name}_interpmv_vectors.hex ({NFRAG} frags x {NVEC} vec4)")

if __name__ == "__main__":
    for f in (FP16, FP24):
        report(f)
        emit_sfu_vectors(f)
        emit_sincos_vectors(f)
        emit_cluster_vectors(f)
    emit_interp_vectors(FP24)
    emit_interp_mv(FP24)
    emit_drawpath(FP24)
    emit_clipnear(FP24)
    emit_primpipe_clip(FP24)
    emit_vtxsetup(FP24, 320.0, 320.0, -240.0, 240.0)
    emit_primpipe(FP24)
    print("\nartifacts in tools/gen/  (load seeds via $readmemh in float_sfu)")
