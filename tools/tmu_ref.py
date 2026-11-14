#!/usr/bin/env python3
"""tmu_ref.py -- bit-exact reference model of the texture unit tmu24 + test vectors.

The model defines the arithmetic the RTL performs (fixed-point coordinates, cube face selection and projection, LOD,
ES 2.0 mip / filter selection, wrap modes, texel decode, filter sums, float conversion), and checks every result
against the exact rational value of the ES 2.0 definition (sections 3.7.5 - 3.7.10, 3.8.2) at the precision the
model's approximations allow (filtered colours within 1 ULP, nearest unorm texels exact).

  ES 2.0 rules implemented:
  * 2D (s, t) and cube maps (direction (x, y, z); major axis, table 3.11, s = (sc/|ma| + 1)/2).
  * lambda = log2(rho) + bias (texture2D(s, c, bias): mode 1, bias = coord.w), or lambda = lod (mode 2, coord.w);
    rho = max(|du/dx|, |du/dy|, |dv/dx|, |dv/dy|) from the quad's differences (allowed by eq. 3.16's bounds);
    for cube maps the differences are taken in the face of pixel 0 (the other pixels' directions projected onto it).
  * magnification iff lambda <= c, c = 0.5 for MAG LINEAR with MIN NEAREST_MIPMAP_*, else 0 (3.7.8).
  * *_MIPMAP_NEAREST: d = 0 (lambda <= 1/2), ceil(lambda + 1/2) - 1, q (lambda > q + 1/2)  (eq. 3.17);
    *_MIPMAP_LINEAR: d1 = floor(lambda) / d2 = d1 + 1, both q when lambda >= q, blend by frac(lambda) (3.18, 3.19).
  * REPEAT = frac(s); MIRRORED_REPEAT = mirror then clamp; CLAMP_TO_EDGE: s in [1/2N, 1 - 1/2N] (3.7.6), any N
    (OES_texture_npot), negative coordinates wrap.  NEAREST i = floor(u); LINEAR i0 = floor(u - 1/2) (mod N for
    REPEAT), i1 = i0 + 1, weights frac(u - 1/2) with 8 fraction bits.
  * texel components of n bits are c / (2^n - 1) (RGBA8, RGBA4444, RGBA5551, RGB565 with alpha 1).
  * incomplete texture (descriptor flag, set by the driver per 3.8.2) -> (0, 0, 0, 1).
  Beyond ES 2.0, OES_texture_3D (s, t, r; descriptor flag vol, depth d <= 255, w, h <= 256): rho also takes |dw/dx|, |dw/dy|;
  the depth halves with the level like w and h; LINEAR = two 4-tap jobs per level, one per slice, in the accumulation
  trilinear uses. Weights: c = 8 fraction bits of r as for s, t; slice k0 gets (lw (256 - c) + 128) >> 8 of the level
  weight lw, slice k1 the rest, so a level's two jobs sum to lw exactly. With one level lw = 256 and this is exact; with
  two levels (3D LINEAR_MIPMAP_LINEAR) the combined weight is rounded to 1/256 (the filter
  and the float conversion keep their widths).

Run:  python3 tools/tmu_ref.py [fp24|fp16] [seed] [--vol3d]   -> tools/gen/tmu24_<fmt>_*.mem + self-check report
      (--vol3d: 3D textures among the descriptors, for a DUT built with VOL3D = 1; the board is built with VOL3D = 0)
"""
import os, sys, random, math
from fractions import Fraction as Fr
sys.path.insert(0, os.path.dirname(__file__))
import float_ref as fr

GEN = os.path.join(os.path.dirname(__file__), "gen")
FRAC = 24                      # fixed-point fraction bits of coordinates
LIMC = 1 << 41                 # coordinate saturation (|s| <= 2^17)
LIMQ8 = 8191                   # lambda / bias / lod clamp (Q.8)
NEG_INF = -(1 << 20)           # lambda of rho == 0
FMT_BPP32 = {0: True, 1: False, 2: False, 3: False}   # 0 RGBA8, 1 RGBA4444, 2 RGBA5551, 3 RGB565
# per format: (shift, bits) of R, G, B, A in the texel (A bits 0 = constant 1.0)
FMT_CH = {0: ((24, 8), (16, 8), (8, 8), (0, 8)),
          1: ((12, 4), (8, 4), (4, 4), (0, 4)),
          2: ((11, 5), (6, 5), (1, 5), (0, 1)),
          3: ((11, 5), (5, 6), (0, 5), (0, 0))}
KD_SH = {8: 8, 4: 4, 5: 5, 6: 6, 1: 0}
# ceil(2^(32+s) / (2^n - 1)) (s = KD_SH[n]): the product Q = Pn * KD over-estimates by < 2^32, less than the distance of
# a non-exact quotient to any rounding boundary (>= 2^(32+s) / (2^n - 1)), so RNE of Q is correct; an exact quotient
# (D divides P) shows as zero bits [31+s:32] of Q, and its error term is then dropped (exact ties round to even)
KD = {n: -((-(1 << (32 + s))) // ((1 << n) - 1)) for n, s in KD_SH.items()}
# log2(1 + k/256) in 1/256: exact at powers of two (lambda = 0 at a 1:1 mapping must stay magnification)
LOG2T = [round(256 * math.log2(1 + k / 256)) for k in range(256)]

def fbits(f, sgn, e, m): return (sgn << (f.W - 1)) | (e << f.M) | m
def fsplit(b, f): return (b >> (f.W - 1)) & 1, (b >> f.M) & ((1 << f.E) - 1), b & ((1 << f.M) - 1)
def fneg(b, f): return b ^ (1 << (f.W - 1))
def fmag(b, f): return b & ((1 << (f.W - 1)) - 1)

def fix(b, f, fracb, lim):
    """float -> fixed point with fracb fraction bits, rounded toward -inf, saturated to +-lim (Inf/NaN saturate)"""
    s, e, m = fsplit(b, f)
    if e == 0: return 0
    if e == (1 << f.E) - 1: return -lim if s else lim
    mant = (1 << f.M) | m
    k = e - f.bias - f.M + fracb
    if k >= 0:
        mag = lim if k > 60 else min(mant << k, lim)
        return -mag if s else mag
    sh = -k
    if s: return -min((mant + (1 << sh) - 1) >> sh, lim)      # floor of a negative value
    return min(mant >> sh, lim)

def bitlen(x): return x.bit_length()

# ---------------------------------------------------------------- descriptor helpers
def level_dims(d, l): return max(1, d['w'] >> l), max(1, d['h'] >> l)
def level_depth(d, l): return max(1, d['d'] >> l)
def level_offsets(d):
    """word offset of each level (0..15) and the chain size (all `levels` levels) of one face"""
    offs = []; o = 0
    for l in range(16):
        offs.append(o)
        if l < d['levels']:
            w, h = level_dims(d, l)
            sz = w * h * level_depth(d, l)
            o += sz if FMT_BPP32[d['fmt']] else (sz + 1) >> 1
    chain = offs[d['levels']] if d['levels'] < 16 else o
    return offs, (chain + 7) & ~7          # the face stride: the chain rounded up to a 32-byte burst (img_geom's layout)

# ---------------------------------------------------------------- cube maps
CUBE_TAB = {0: ('z', 1, 'y', 1, 'x'), 1: ('z', 0, 'y', 1, 'x'), 2: ('x', 0, 'z', 0, 'y'),
            3: ('x', 0, 'z', 1, 'y'), 4: ('x', 0, 'y', 1, 'z'), 5: ('x', 1, 'y', 1, 'z')}   # (sc comp, negate, tc comp, negate, ma)

def cube_face(r, f):
    ax, ay, az = fmag(r['x'], f), fmag(r['y'], f), fmag(r['z'], f)
    if ax >= ay and ax >= az: c = 'x'; base = 0
    elif ay >= az:            c = 'y'; base = 2
    else:                     c = 'z'; base = 4
    return base + ((r[c] >> (f.W - 1)) & 1)

def cube_div(a_mant, M):
    """the RTL's pipelined restoring divider: floor(2^(M+26) / A), A in [2^M, 2^(M+1))"""
    return (1 << (M + 26)) // a_mant

def cube_q(sc, ma_mant, ma_e, f, R):
    """q = sc / |ma| as floor(|q| * 2^25) with the sign of sc (P = S * R, shifted by M + 1 + ea - es)"""
    s, e, m = fsplit(sc, f)
    if e == 0: return 0, 0
    S = (1 << f.M) | m
    P = S * R
    sh = f.M + 1 + ma_e - e
    qq = P >> sh if sh >= 0 else min(P << (-sh), LIMC)
    return s, min(qq, LIMC)

def cube_st(r, face, f, proj):
    """(sfix, tfix) of direction r in `face`; proj = projection of another pixel onto pixel 0's face (LOD only)"""
    scc, scn, tcc, tcn, mac = CUBE_TAB[face]
    sc = fneg(r[scc], f) if scn else r[scc]
    tc = fneg(r[tcc], f) if tcn else r[tcc]
    ma = r[mac]
    ms, me, mm = fsplit(ma, f)
    if me == 0 or (proj and ms != (face & 1)):          # zero major / pointing away from the face
        return (LIMC, LIMC) if proj else (1 << 23, 1 << 23)
    if me == (1 << f.E) - 1: me = (1 << f.E) - 2         # Inf / NaN major: largest finite exponent
    R = cube_div((1 << f.M) | mm, f.M)
    out = []
    for c in (sc, tc):
        sg, qq = cube_q(c, (1 << f.M) | mm, me, f, R)
        v = (-qq if sg else qq) + (1 << 25)
        out.append(max(-LIMC, min(LIMC, v >> 2)))
    return out[0], out[1]

# ---------------------------------------------------------------- LOD
def lam_term(dlt, dim):
    """lambda (Q.8) of |dlt| * dim, dlt a Q.24 difference: 18-bit mantissa x dim, log2 of the product (table)"""
    d = abs(dlt)
    if d == 0: return NEG_INF
    ed = bitlen(d) - 1
    m18 = d >> (ed - 17) if ed >= 17 else d << (17 - ed)
    t = m18 * dim
    mt = bitlen(t) - 1
    k8 = (t >> (mt - 8)) & 255
    return (ed - 17 + mt - FRAC) * 256 + LOG2T[k8]

def clampq8(v): return max(-LIMQ8, min(LIMQ8, v))

def select_levels(d, lam):
    """per pixel: list of (level, level weight /256) and linear flag"""
    magl, mip, minl = (d['filt'] >> 3) & 1, (d['filt'] >> 1) & 3, d['filt'] & 1
    q = max(0, d['levels'] - 1)
    c = 128 if (magl and not minl and mip in (1, 2)) else 0
    if lam <= c:
        return [(0, 256)], magl
    if mip == 0: return [(0, 256)], minl
    if mip == 1:
        dd = 0 if lam <= 128 else min(q, (lam + 127) >> 8)
        return [(dd, 256)], minl
    # mip == 2 (or 3): linear between levels
    if lam >= q * 256: return [(q, 256)], minl
    d1 = lam >> 8; fr8 = lam & 255
    if fr8 == 0: return [(d1, 256)], minl
    return [(d1, 256 - fr8), (d1 + 1, fr8)], minl

# ---------------------------------------------------------------- taps
def wrap_u(sfix, N, wrap):
    """u in Q.24 texel units after the wrap mode (REPEAT, CLAMP, MIRRORED)"""
    if wrap == 1:                                          # CLAMP_TO_EDGE
        sp = min(max(sfix, 0), 1 << FRAC)
    elif wrap == 2:                                        # MIRRORED_REPEAT: mirror, then clamp
        mm = sfix % (1 << (FRAC + 1))
        sp = mm if mm < (1 << FRAC) else (1 << (FRAC + 1)) - mm
    else:                                                  # REPEAT (and 3)
        return (sfix % (1 << FRAC)) * N
    u = sp * N
    return min(max(u, 1 << (FRAC - 1)), N * (1 << FRAC) - (1 << (FRAC - 1)))

def axis_taps(sfix, N, wrap, lin):
    """(i0, i1, alpha8): NEAREST -> (i, i, 0)"""
    u = wrap_u(sfix, N, wrap)
    if not lin:
        return u >> FRAC, u >> FRAC, 0
    u2 = u - (1 << (FRAC - 1))
    i0 = u2 >> FRAC; a = (u2 >> (FRAC - 8)) & 255
    if wrap in (0, 3):
        if i0 < 0: i0 += N
        i1 = i0 + 1
        if i1 >= N: i1 -= N
    else:
        i1 = min(i0 + 1, N - 1)
    return i0, i1, a

def taps_of(d, face, sfix, tfix, rfix, level, lw, lin, offs, chain):
    """the taps of one pixel and level: (word address, half, weight); 4, or 8 for 3D LINEAR (two jobs, one per slice)"""
    N, M = level_dims(d, level)
    i0, i1, a = axis_taps(sfix, N, d['wrap_s'], lin)
    j0, j1, b = axis_taps(tfix, M, d['wrap_t'], lin)
    if lin: ws = [(256 - a) * (256 - b), a * (256 - b), (256 - a) * b, a * b]
    else:   ws = [65536, 0, 0, 0]
    k0, k1, c = axis_taps(rfix, level_depth(d, level), d['wrap_r'], lin) if d['vol'] else (0, 0, 0)
    m = (lw * (256 - c) + 128) >> 8
    jobs = [(k0, m), (k1, lw - m)] if d['vol'] and lin else [(k0, m)]
    out = []
    for k, jw in jobs:
        for (i, j), w in zip(((i0, j0), (i1, j0), (i0, j1), (i1, j1)), ws):
            T = (k * M + j) * N + i
            word = (d['base'] + face * chain + offs[level] + (T if FMT_BPP32[d['fmt']] else T >> 1)) & ((1 << 29) - 1)
            out.append((word, T & 1, w * jw))
    return out

def texel(mem, d, word, half):
    v = mem.get(word, 0)
    if not FMT_BPP32[d['fmt']]: v = (v >> 16) & 0xFFFF if half else v & 0xFFFF
    return v

# ---------------------------------------------------------------- conversion
def to_float(P, n, f):
    """P / ((2^n - 1) * 2^24) -> float bits, the RTL's way: normalise P to 32 bits, multiply by KD, RNE"""
    if P == 0: return 0
    msb = bitlen(P) - 1
    Pn = P << (31 - msb)
    Q = Pn * KD[n]
    s = KD_SH[n]
    if ((Q >> 32) & ((1 << s) - 1)) == 0:                 # exact quotient: drop the over-estimate
        Q = (Q >> (32 + s)) << (32 + s)
    Lq = bitlen(Q) - 1
    mant = Q >> (Lq - f.M)
    guard = (Q >> (Lq - f.M - 1)) & 1
    sticky = (Q & ((1 << (Lq - f.M - 1)) - 1)) != 0
    if guard and (sticky or (mant & 1)): mant += 1
    if mant == 1 << (f.M + 1): mant >>= 1; Lq += 1
    e = Lq + msb - 31 - FRAC - 32 - KD_SH[n] + f.bias
    if e <= 0: return 0
    return (e << f.M) | (mant & ((1 << f.M) - 1))

# ---------------------------------------------------------------- one quad
COV = {}
def cov(k): COV[k] = COV.get(k, 0) + 1

def sample(d, mem, req, f, exact_check=None):
    """req: {'samp', 'mode', 'c': [(x, y, z, w) bits x 4]} -> 16 result bits (pixel p, component c at p*4+c)"""
    one = fr.encode_real(Fr(1), f)
    cov('quads'); cov(f"fmt{d['fmt']}"); cov(f"mode{req['mode']}")
    if d['incomplete']:
        cov('incomplete')
        return [0, 0, 0, one] * 4
    if d['cube']: cov('cube')
    if d['vol']: cov('vol')
    if (d['w'] & (d['w'] - 1)) or (d['h'] & (d['h'] - 1)): cov('npot')
    offs, chain = level_offsets(d)
    px = req['c']
    if d['cube']:
        rs = [dict(x=c[0], y=c[1], z=c[2]) for c in px]
        faces = [cube_face(r, f) for r in rs]
        st = [cube_st(r, fc, f, False) for r, fc in zip(rs, faces)]
        stl = [st[p] if faces[p] == faces[0] else cube_st(rs[p], faces[0], f, True) for p in range(4)]
        N0 = d['w']; M0 = d['w']
        if any(fc != faces[0] for fc in faces): cov('cube_cross')
        for fc in faces: cov(f"face{fc}")
    else:
        faces = [0] * 4
        st = [(fix(c[0], f, FRAC, LIMC), fix(c[1], f, FRAC, LIMC)) for c in px]
        stl = st
        N0 = d['w']; M0 = d['h']
    # LOD from the quad (pixels 0 TL, 1 TR, 2 BL)
    terms = [lam_term(stl[1][0] - stl[0][0], N0), lam_term(stl[2][0] - stl[0][0], N0),
             lam_term(stl[1][1] - stl[0][1], M0), lam_term(stl[2][1] - stl[0][1], M0)]
    rf = [fix(c[2], f, FRAC, LIMC) for c in px]
    if d['vol']: terms += [lam_term(rf[1] - rf[0], d['d']), lam_term(rf[2] - rf[0], d['d'])]
    lbase = max(terms)
    out = []
    for p in range(4):
        mode = req['mode']
        if mode == 2:   lam = clampq8(fix(px[p][3], f, 8, LIMQ8))
        elif lbase == NEG_INF: lam = NEG_INF
        elif mode == 1: lam = clampq8(lbase + fix(px[p][3], f, 8, LIMQ8))
        else:           lam = clampq8(lbase)
        levels, lin = select_levels(d, lam)
        magl, mip, minl = (d['filt'] >> 3) & 1, (d['filt'] >> 1) & 3, d['filt'] & 1
        c = 128 if (magl and not minl and mip in (1, 2)) else 0
        cov('px_mag' if lam <= c else f'px_min_mip{mip}')
        if c and 0 < lam <= c: cov('px_c_half_rule')
        if len(levels) == 2: cov('px_trilinear')
        if levels[0][0] > 0: cov('px_level>0')
        cov('px_linear' if lin else 'px_nearest')
        if not d['cube']:
            for ax, wr in ((0, d['wrap_s']), (1, d['wrap_t'])):
                cov(f"wrap{wr}");
                if st[p][ax] < 0: cov(f"neg_coord_wrap{wr}")
                if st[p][ax] > (1 << FRAC): cov(f"gt1_coord_wrap{wr}")
        if d['vol']:
            cov(f"vol_wrap{d['wrap_r']}"); cov('vol_linear' if lin else 'vol_nearest')
            if len(levels) == 2: cov('vol_trilinear')
        P = [0, 0, 0, 0]
        for lvl, lw in levels:
            for word, half, w in taps_of(d, faces[p], st[p][0], st[p][1], rf[p], lvl, lw, lin, offs, chain):
                tv = texel(mem, d, word, half)
                for ch, (sh, nb) in enumerate(FMT_CH[d['fmt']]):
                    if nb: P[ch] += w * ((tv >> sh) & ((1 << nb) - 1))
        res = []
        for ch, (sh, nb) in enumerate(FMT_CH[d['fmt']]):
            res.append(one if nb == 0 else to_float(P[ch], nb, f))
            if exact_check is not None and nb:
                exact = Fr(P[ch], ((1 << nb) - 1) << FRAC)
                exact_check(res[-1], exact, f)
        out += res
    return out

# ---------------------------------------------------------------- vectors
def rnd_float(f, v):
    return fr.encode_real(Fr(v).limit_denominator(1 << 30), f) if v != 0 else 0

def make_texture(rng, mem, base, d):
    offs, chain = level_offsets(d)
    nfaces = 6 if d['cube'] else 1
    words = chain * nfaces
    style = rng.randrange(3)
    for k in range(words):
        if style == 0:   v = rng.getrandbits(32)
        elif style == 1: v = rng.choice([0, 0xFFFFFFFF, 0x80808080, 0x7F7F7F7F, rng.getrandbits(32)])
        else:            v = (k * 0x01030507 + 0x10203040) & 0xFFFFFFFF
        mem[base + k] = v
    return words

def make_desc(rng, base, cube=None, vol=False):
    cube = rng.random() < 0.25 if cube is None else cube
    dep = 1
    if cube:
        w = rng.choice([1, 2, 4, 8, 16, 5, 12]); h = w
    elif vol:
        w = rng.choice([1, 2, 3, 4, 8, 16, 17, 32]); h = rng.choice([1, 2, 4, 5, 8, 16]); dep = rng.choice([1, 2, 3, 4, 7, 8, 16, 21])
    else:
        w = rng.choice([1, 2, 3, 4, 5, 7, 8, 16, 17, 32, 33, 64, 100, 2048]); h = rng.choice([1, 2, 4, 6, 8, 16, 31, 32, 64])
        if w == 2048: h = rng.choice([1, 2, 4, 8])            # MAX_TEXTURE_SIZE (12 levels); keeps the bench memory small
    full = bitlen(max(w, h, dep))
    levels = rng.randint(1, full)
    mag = rng.randrange(2); mip = rng.randrange(3); minl = rng.randrange(2)
    return dict(base=base, w=w, h=h, d=dep, fmt=rng.randrange(4), levels=levels, filt=(mag << 3) | (mip << 1) | minl,
                wrap_s=rng.choice([0, 1, 2, 0]), wrap_t=rng.choice([0, 1, 2, 1]), wrap_r=rng.choice([0, 1, 2]), cube=int(cube),
                vol=int(vol), incomplete=int(rng.random() < 0.05))

def make_req(rng, d, f, samp):
    mode = rng.choice([0, 0, 0, 1, 2])
    if d['cube']:
        x, y, z = (rng.uniform(-1, 1) for _ in range(3))
        if rng.random() < 0.2: x, y, z = rng.choice([(1, 0.999, 0.2), (-0.5, 0.5, 0.5), (0.3, -0.3, -0.3)])
        scale = rng.choice([1, 1, 3.5, 0.01, 100])
        dx = rng.choice([0, 0.001, 0.01, 0.05, 0.2]) ; dy = rng.choice([0, 0.001, 0.02, 0.1])
        base = [(x, y, z), (x + dx, y, z - dx), (x, y + dy, z), (x + dx, y + dy, z - dx)]
        cs = [(c[0] * scale, c[1] * scale, c[2] * scale) for c in base]
    else:
        s0 = rng.choice([rng.uniform(-0.2, 1.2), rng.uniform(-3, 3), rng.uniform(-1000, 1000), 0.0, 1.0, 0.5,
                         -1e-7, 1.0 - 1e-6, rng.uniform(0, 1)])
        t0 = rng.choice([rng.uniform(-0.2, 1.2), rng.uniform(-3, 3), 0.0, 1.0, rng.uniform(0, 1)])
        sc = rng.choice([0, 2 ** -12, 2 ** -8, 2 ** -6, 2 ** -4, 2 ** -2, 1, 4]) * rng.choice([1, -1, 0.7])
        tc = rng.choice([0, 2 ** -12, 2 ** -8, 2 ** -5, 2 ** -3, 1]) * rng.choice([1, -1, 1.3])
        # r: read by 3D samplers; on 2D ones it is whatever the shader's register held and must change nothing
        r0 = rng.choice([rng.uniform(-0.2, 1.2), rng.uniform(-3, 3), 0.0, 1.0, 0.5, rng.uniform(0, 1)])
        rc = rng.choice([0, 0, 2 ** -8, 2 ** -5, 2 ** -3, 1]) * rng.choice([1, -1, 0.6])
        cs = [(s0, t0, r0), (s0 + sc, t0 + sc * 0.3, r0 + rc), (s0 - tc * 0.2, t0 + tc, r0 - rc * 0.4),
              (s0 + sc - tc * 0.2, t0 + sc * 0.3 + tc, r0 + rc * 0.6)]
    ws = [rng.choice([0, 0.5, -0.5, 1.25, 2, -3, 7.75, 0.4999]) for _ in range(4)]
    return dict(samp=samp, mode=mode, c=[tuple(rnd_float(f, v) for v in (c[0], c[1], c[2], w)) for c, w in zip(cs, ws)])

def main():
    vol3d = '--vol3d' in sys.argv                          # 3D textures among the descriptors: only for a DUT built with VOL3D = 1
    args = [a for a in sys.argv[1:] if a != '--vol3d']      # (tb_tmu24 -Defines TB_VOL3D); the board is built with VOL3D = 0
    fname = args[0] if len(args) > 0 else 'fp24'
    seed = int(args[1]) if len(args) > 1 else 1
    f = fr.FP24 if fname == 'fp24' else fr.FP16
    rng = random.Random(seed)
    mem = {}
    MEMW = 1 << 19                                     # bench memory: 2^19 words
    stats = dict(n=0, exact=0, off1=0, bad=0, maxulp=0)
    def check(bits, exact, f):
        """every channel must be the correctly rounded (RNE, FTZ) exact ES 2.0 value"""
        stats['n'] += 1
        if bits == fr.encode_real(exact, f): stats['exact'] += 1; return
        u = fr.ulp_err(bits, exact, f)
        stats['maxulp'] = max(stats['maxulp'], u)
        if u <= 1.0: stats['off1'] += 1
        else: stats['bad'] += 1
    # 2 phases of 16 samplers (the bench reloads the descriptors between them)
    phases = []
    base = 0x100
    for ph in range(2):
        descs = []
        for s in range(16):
            d = make_desc(rng, base, cube=(s % 4 == 3), vol=(vol3d and s % 4 == 2))
            words = make_texture(rng, mem, base, d)
            base += words + rng.randrange(1, 40)
            descs.append(d)
        reqs = []
        for k in range(300 if ph == 0 else 200):
            s = rng.randrange(16)
            reqs.append(make_req(rng, descs[s], f, s))
        # throughput block: one bilinear 2D sampler, quads across the texture
        if ph == 1:
            d = dict(base=base, w=64, h=64, d=1, fmt=0, levels=1, filt=0b1001, wrap_s=0, wrap_t=0, wrap_r=0, cube=0, vol=0, incomplete=0)
            base += make_texture(rng, mem, base, d) + 8
            descs[0] = d
            for rep in range(2):                           # twice: a cold and a warm (cached) pass
                for k in range(256):
                    x = (k % 16) * 4 / 64.0; y = (k // 16) * 4 / 64.0; e = 1 / 64.0
                    cs = [(x, y), (x + e, y), (x, y + e), (x + e, y + e)]
                    reqs.append(dict(samp=0, mode=0, c=[(rnd_float(f, a), rnd_float(f, b), 0, 0) for a, b in cs]))
        phases.append((descs, reqs))
    assert base < MEMW
    tag = f"tmu24_{fname}"
    hw = (f.W + 3) // 4
    with open(f"{GEN}/{tag}_mem.mem", "w") as fo:
        for a in range(MEMW): fo.write(f"{mem.get(a, 0):08x}\n")
    with open(f"{GEN}/{tag}_desc.mem", "w") as fo:          # per phase 16 lines: {vol,wrapr,depth,inc,cube,wrapt,wraps,filt,lvls,fmt,h,w,base}
        for descs, _ in phases:
            for d in descs:
                v = (d['vol'] << 83) | (d['wrap_r'] << 81) | (d['d'] << 73) | \
                    (d['incomplete'] << 72) | (d['cube'] << 71) | (d['wrap_t'] << 69) | (d['wrap_s'] << 67) | \
                    (d['filt'] << 63) | (d['levels'] << 59) | (d['fmt'] << 57) | (d['h'] << 45) | (d['w'] << 33) | d['base']
                fo.write(f"{v:021x}\n")
    nreq = []
    with open(f"{GEN}/{tag}_req.mem", "w") as fr_, open(f"{GEN}/{tag}_exp.mem", "w") as fe:
        for descs, reqs in phases:
            nreq.append(len(reqs))
            for r in reqs:
                bits = 0
                for p in range(4):
                    for c in range(4): bits |= r['c'][p][c] << ((p * 4 + c) * f.W)
                fr_.write(f"{(((r['samp'] << 2) | r['mode']) << (16 * f.W)) | bits:x}\n")   # {samp, mode, 16 coords}
                res = sample(descs[r['samp']], mem, r, f, check)
                ebits = 0
                for k, v in enumerate(res): ebits |= v << (k * f.W)
                fe.write(f"{ebits:0{16 * hw}x}\n")
    with open(f"{GEN}/{tag}_n.mem", "w") as fo:
        for n in nreq: fo.write(f"{n:x}\n")
    print(f"{tag}: {sum(nreq)} quads in {len(phases)} phases; channels vs the exact value: {stats['n']} -- "
          f"correctly rounded {stats['exact']}, 1 ULP off {stats['off1']}, worse {stats['bad']}")
    print("  coverage:", ", ".join(f"{k} {v}" for k, v in sorted(COV.items())))
    if stats['bad'] or stats['off1']: sys.exit(1)

if __name__ == "__main__":
    main()
