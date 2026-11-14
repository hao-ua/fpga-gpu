#!/usr/bin/env python3
"""Conservative depth bounds of a quad — the bit-exact model of zq_tri.sv (board A's setup: four words per primitive in the
TRI item) and zq_lane.sv (per quad: board B's early depth test ahead of quad_setup, board A's coarse-depth culling).
The derivation, in short:

B's quad_setup forms z(p) = MAD(mu2, dz2, MAD(mu1, dz1, z0)) with mu_k = MUL(i2f(e_k(p)), R) and d = clamp(rne(z D) + o24)
(D = 2^24 - 1 or 2^16 - 1). For a COVERED pixel 0 <= mu_k <= 1 (+ a few ulps), and against the exact plane
T(p) = z0 + R (e1(p) dz1 + e2(p) dz2) (B's own R, exact integers e) the hardware z is off by at most
u (2 |z0| + 5 |dz1| + 4 |dz2|), u = 2^-17. Per primitive (zq_words):
    k  = eR - 47                       e' = floor(e 2^k) holds |e| <= |a2 >> s| ~ 1/R in 17 bits (|e'| <= 2^16 covered)
    m_j = round(R 2^-k dz_j 2^24 2^14)  = round(mR mdj 2^(edj - 73)), 25-bit signed: the plane's slope per e' unit
    Sx = A1' m1 + A2' m2, Sy = B1' m1 + B2' m2   the step per pixel (2^-14 Z units, Z = z 2^24)
    Z0 = floor(z0 2^24)
    M  = 2^(e0-54) + 2^(ed1-50) + 2^(ed2-50) + 16 >= the hardware error + this model's (Z units)
    Clo = Z0 + floor(min(Sx, 0) / 2^14) + floor(min(Sy, 0) / 2^14) - M
    Chi = Z0 + floor(max(Sx, 0) / 2^14) + floor(max(Sy, 0) / 2^14) + M + 4
Per quad (zq_quad), from its pixel-0 edge values e1, e2 (what A sends): S0 = e1' m1 + e2' m2,
    Lo = Clo + floor(S0 / 2^14) - 3, Hi = Chi + floor(S0 / 2^14) + 5 (24 bits); for 16 bits (Lo - 1280) >> 8, (Hi + 1536) >> 8
    [Dlo, Dhi] = [clamp(Lo), clamp(Hi)] holds d of every covered pixel of the quad (polygon offset 0).
Not valid (ok = 0): lines (their pixels' mu may leave [0, 1]), R outside [2^-30, 1], z0 < 0, |z0| or |dz| >= 2, a step
that does not fit 18 bits after the shift; per quad: e' beyond 18 bits.

CLI: python3 tools/zq_ref.py check [n]   random triangles, every covered pixel's d against its quad's bounds
"""
import os, sys, random
from fractions import Fraction as F
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import b_ref

MASK32 = 0xFFFFFFFF


def fp(x):
    """FP24 bits -> (sign, exponent, 17-bit significand or 0)"""
    e = (x >> 16) & 0x7F
    return (x >> 23) & 1, e, ((x & 0xFFFF) | 0x10000) if e else 0


def shk(v, k):
    """floor(v 2^k)"""
    return v << k if k >= 0 else v >> (-k)


def fits(v, bits): return -(1 << (bits - 1)) <= v < (1 << (bits - 1))


def sx(v, bits):
    v &= (1 << bits) - 1
    return v - (1 << bits) if v >> (bits - 1) else v


def slope(R, dz):
    """m = round(mR md 2^(ed - 73)) with the sign of R dz (round half up on the magnitude)"""
    sR, eR, mR = fp(R); sd, ed, md = fp(dz)
    if ed == 0: return 0
    sh = 73 - ed
    P = mR * md
    mag = 0 if sh >= 36 else (P + (1 << (sh - 1))) >> sh
    return -mag if (sR ^ sd) else mag


def zq_words(R, z0, dz1, dz2, A1, B1, A2, B2, line=False):
    """the four TRI item words w18..w21: {ok, k[30:25], m1[24:0]}, m2[24:0], Clo, Chi"""
    sR, eR, mR = fp(R); s0, e0, m0 = fp(z0); _, ed1, _ = fp(dz1); _, ed2, _ = fp(dz2)
    k = eR - 47
    ok = (not line) and 33 <= eR <= 63 and e0 <= 63 and not (s0 and e0) and ed1 <= 63 and ed2 <= 63
    m1, m2 = slope(R, dz1), slope(R, dz2)
    st = [shk(sx(v, 32), k) for v in (A1, A2, B1, B2)]
    ok = ok and all(fits(v, 18) for v in st)
    if not ok: return [0, 0, 0, 0]
    Sx = st[0] * m1 + st[1] * m2
    Sy = st[2] * m1 + st[3] * m2
    Z0 = 0 if e0 == 0 else shk(m0, e0 - 55)
    M = (1 << max(0, e0 - 54)) + (1 << max(0, ed1 - 50)) + (1 << max(0, ed2 - 50)) + 16
    Clo = Z0 + (min(Sx, 0) >> 14) + (min(Sy, 0) >> 14) - M
    Chi = Z0 + (max(Sx, 0) >> 14) + (max(Sy, 0) >> 14) + M + 4
    assert fits(Clo, 32) and fits(Chi, 32) and fits(m1, 25) and fits(m2, 25)
    return [(1 << 31) | ((k & 0x3F) << 25) | (m1 & 0x1FFFFFF), m2 & 0x1FFFFFF, Clo & MASK32, Chi & MASK32]


def zq_quad(w, e1, e2, d16=0):
    """(valid, Dlo, Dhi) of a quad from its pixel-0 edge values (32-bit words as sent)"""
    ok = (w[0] >> 31) & 1
    k = sx(w[0] >> 25, 6); m1 = sx(w[0], 25); m2 = sx(w[1], 25); Clo = sx(w[2], 32); Chi = sx(w[3], 32)
    a, b = shk(sx(e1, 32), k), shk(sx(e2, 32), k)
    if not ok or not fits(a, 18) or not fits(b, 18): return 0, 0, 0
    s = (a * m1 + b * m2) >> 14
    if d16: lo, hi, dmax = (Clo + s - 1280) >> 8, (Chi + s + 1536) >> 8, 0xFFFF
    else:   lo, hi, dmax = Clo + s - 3, Chi + s + 5, 0xFFFFFF
    return 1, max(0, min(dmax, lo)), max(0, min(dmax, hi))


def tri_words(t):
    """zq_words of a b_ref triangle dict (tri_consts with P)"""
    AB = t['AB']
    return zq_words(t['R'], t['z0'], t['dz1'], t['dz2'], AB[2], AB[3], AB[4], AB[5])


def check(n=3000, seed=1):
    """every covered pixel's quad_setup depth inside its quad's bounds, over random triangles of every size and shape"""
    rng = random.Random(seed)
    bad = tested = valid = 0; width = []; slack = []; worst = F(0)
    u = F(1, 1 << 17)
    for i in range(n):
        kind = i % 4
        if kind == 0:
            P, a2, z, q, zs, ws = b_ref.rand_tri(rng)                        # anywhere on a 2048 x 2048 screen
        else:                                                                # small / thin triangles near a point
            sz = [2, 16, 200][kind - 1]
            cx, cy = rng.randrange(sz + 8, sz + 600), rng.randrange(sz + 8, sz + 400)
            while True:
                P = [(F(cx * 16 + rng.randrange(-sz * 16, sz * 16), 16), F(cy * 16 + rng.randrange(-sz * 16, sz * 16), 16)) for _ in range(3)]
                a2 = (P[1][0] - P[0][0]) * (P[2][1] - P[0][1]) - (P[2][0] - P[0][0]) * (P[1][1] - P[0][1])
                if a2 != 0: break
            zs = [rng.random() for _ in range(3)] if rng.randrange(3) else [0.97 + 0.03 * rng.random() for _ in range(3)]
            z = [b_ref.enc(F(v)) for v in zs]; q = [b_ref.ONE] * 3
        t = b_ref.tri_consts(z, q, 0, 0, P)
        w = tri_words(t)
        d16 = rng.randrange(4) == 0; bits = 16 if d16 else 24
        AB, a2r = b_ref.edge_steps_raw(P)
        xs = [int(p[0]) for p in P]; ys = [int(p[1]) for p in P]
        for _ in range(12):                                                  # quads with a covered pixel
            px, py = rng.randrange(max(0, min(xs) - 1), max(xs) + 2), rng.randrange(max(0, min(ys) - 1), max(ys) + 2)
            qx, qy = px >> 1, py >> 1
            cov = b_ref.quad_cover(P, qx, qy, 4096, 4096)
            if not cov: continue
            e = b_ref.pixel_edges(P, t['s'], 2 * qx, 2 * qy)
            ok, lo, hi = zq_quad([v & MASK32 for v in w], e[1] & MASK32, e[2] & MASK32, d16)
            tested += 1
            if not ok: continue
            valid += 1; width.append(hi - lo)
            ds = []
            for p in range(4):
                if not (cov >> p) & 1: continue
                o = b_ref.quad_setup_e(t, e, p, 2 * qx + (p & 1), 2 * qy + (p >> 1), 0, bits)
                d = o['d24']; ds.append(d)
                # the analysis: |z - T| <= u (2 |z0| + 5 |dz1| + 4 |dz2|), T the exact plane with B's R
                zv = b_ref.val(o['z']); R = b_ref.val(t['R'])
                T = b_ref.val(t['z0']) + R * (o['ep'][1] * b_ref.val(t['dz1']) + o['ep'][2] * b_ref.val(t['dz2']))
                eb = u * (2 * abs(b_ref.val(t['z0'])) + 5 * abs(b_ref.val(t['dz1'])) + 4 * abs(b_ref.val(t['dz2'])))
                if eb: worst = max(worst, abs(zv - T) / eb)
                if not lo <= d <= hi:
                    bad += 1
                    if bad < 10: print(f"OUT: tri {i} quad ({qx},{qy}) p{p} d {d} not in [{lo}, {hi}] (d16 {d16}) P {P} z {zs}")
            if ds: slack.append((hi - lo) - (max(ds) - min(ds)))
    width.sort(); slack.sort()
    med = width[len(width) // 2] if width else 0
    print(f"zq_ref check: {tested} quads, {valid} with bounds, {bad} pixels outside; bound width median {med}, "
          f"90 % {width[int(len(width) * 0.9)] if width else 0}; slack over the covered pixels' own spread median "
          f"{slack[len(slack) // 2] if slack else 0}; worst |z - T| / bound {float(worst):.3f} (must stay <= 1)")
    return bad == 0


if __name__ == '__main__':
    a = sys.argv[1:]
    if a and a[0] == 'check':
        sys.exit(0 if check(int(a[1]) if len(a) > 1 else 3000) else 1)
    print(__doc__)
