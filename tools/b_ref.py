#!/usr/bin/env python3
"""Board B pixel side — bit-exact reference model.

FP24 (s1 e7 m16, bias 63, FTZ, RNE) through tools/float_ref.py; MAD = round(mul) then round(add) as float_madd;
RCP = the FP24 SFU (seed K = 12 + one Newton step) as float_sfu.

Per pixel (quad_setup): A sends per QUAD the three integer edge values e0 e1 e2 of the quad's
pixel 0 (in 1/256 pixel^2, right-shifted by the triangle's s, see edge_shift) and per triangle the per-pixel steps
A_k (x) and B_k (y) of each edge and R = RCP(i2f(a2 >> s)); B forms pixel p's values e_k(p) = e_k + A_k (p & 1) +
B_k (p >> 1) (32-bit integer adds) and mu_k = MUL(i2f(e_k(p)), R) — the same expression on A, B and here:
    z   = MAD(mu2, dz2, MAD(mu1, dz1, z0))          window z, screen-linear (delta form: a constant depth is exact)
    w_f = MAD(mu2, q2, MAD(mu1, q1, MUL(mu0, q0)))  gl_FragCoord.w = interpolated 1/w_c (positive terms: no
                                                    cancellation, so the relative error stays small for any w ratio)
    l_k = MUL(MUL(mu_k, q_k), RCP(w_f))             perspective weights (k = 1, 2); MUL(mu_k, q_k) = the MAD's product
    d24 = clamp(rne(z * (2^b - 1)) + o24, 0, 2^b - 1)     b = 24, or 16 for a DEPTH_COMPONENT16 attachment (depth16)
    gl_FragCoord = (x + 0.5, y + 0.5, rne_fp24(d24 / (2^b - 1)), w_f)
Per triangle: o24 = rne(MUL(factor, max(|dz/dx|, |dz/dy|)) * (2^b - 1)) + units, saturated to +-2^25.
LDV (FS core, INTERP): v = MAD(l2, D2, MAD(l1, D1, f0)) per component.
ROP (b_rop, pixel_blend_b): render targets RGBA8 / RGBA4444 / RGBA5551 / RGB565 (fmt 0..3 as tmu24), each channel in
its own units ONE = 16 (2^n - 1); an optional D24S8 depth-stencil image persists depth and stencil across frames.

CLI:
    python3 tools/b_ref.py qs <seed> <ntri> [d16]  -> tools/gen/qs_vectors.hex (tb_quad_setup) + exactness report
    python3 tools/b_ref.py conv                 -> exhaustive check of the u24 <-> FP24 conversions' integer forms
    python3 tools/b_ref.py div                  -> exhaustive check of the blend's divisions (every channel width)
    python3 tools/b_ref.py fsi <prog.sasm>      -> tools/gen/<prog>_fsi_iram.mem + _fsil_iram.mem (tb_fs_interp)
    python3 tools/b_ref.py frame <seed> <ntri>  -> tools/gen/br_*.hex, bl_cmds.hex (tb_b_render, tb_board_b3d); the
        frame's target format / depth bits / depth-stencil image are drawn from the seed, or forced with BR_FMT=0..3,
        BR_D16=0/1, BR_DS=0/1 in the environment; BR_NOCLEAR=1: no clear, BR_CLEARSC=1: a colour clear in its scissor
        box; BR_SKIPBANDS=1: the BAND items name the next
        band and the bands with no quad that the clear does not reach are left out (board A with FLUSH flags bit 10)
    python3 tools/b_ref.py aframe <seed>        -> tools/gen/ab_<seed>_* : board A's draw <seed> (as_<seed>_stream.hex)
        rendered as B must (tb_two_boards3d)
"""
import os, sys, random, math
from fractions import Fraction as F
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import float_ref as fr

f24 = fr.FP24
GEN = fr.GEN
U24 = (1 << 24) - 1
RCP_K = 12
RCP_SEED, RCP_SW = fr.gen_rcp_seed(f24, RCP_K)
ONE = f24.bias << f24.M
HALF = (f24.bias - 1) << f24.M


def mul(a, b): return fr.fmul(a, b, f24)
def add(a, b): return fr.fadd(a, b, f24)
def mad(a, b, c): return fr.madd(a, b, c, f24)


def rcp(x, f=f24, K=RCP_K, seed=RCP_SEED, SW=RCP_SW):
    """float_sfu RCP exactly as the RTL computes it: seed (K index bits, SW fraction bits), one Newton step in
    FW = M + 10 fraction bits with the products truncated (>> FW), then RNE of the truncated result"""
    M, E, bias = f.M, f.E, f.bias
    FW = M + 10
    s = (x >> (E + M)) & 1; ef = (x >> M) & ((1 << E) - 1); mf = x & ((1 << M) - 1)
    if ef == f.inf_exp:
        return fr.qnan(f) if mf else (s << (E + M))                 # 1/inf = +-0
    if ef == 0:
        return (s << (E + M)) | (f.inf_exp << M)                    # 1/0 = +-inf
    e = ef - bias
    sig = (1 << M) | mf
    Y0 = seed[mf >> (M - K)] << (FW - SW)
    Mr = sig << (FW - M)
    myv = (Mr * Y0) >> FW
    dr = ((1 << (FW + 1)) - myv) & ((1 << (FW + 3)) - 1)
    Y1 = ((Y0 * dr) >> FW) & ((1 << (FW + 2)) - 1)
    if (Y1 >> FW) & 1: sv, ex = Y1, -e
    else: sv, ex = Y1 << 1, -e - 1
    mn = (sv >> (FW - M)) & ((1 << M) - 1)
    g = (sv >> (FW - 1 - M)) & 1
    st = (sv & ((1 << (FW - 1 - M)) - 1)) != 0
    mr = mn + (g & (1 if (st or (mn & 1)) else 0))
    if mr >> M: mr = 0; ex += 1
    if ex + bias >= f.inf_exp: return (s << (E + M)) | (f.inf_exp << M)
    if ex + bias <= 0: return s << (E + M)
    return (s << (E + M)) | ((ex + bias) << M) | mr
def enc(v): return fr.encode_real(F(v), f24)
def val(b):
    r = fr.to_real(b, f24)
    return r


def rne_int(q):
    """round a Fraction to the nearest integer, ties to even"""
    fl = q.numerator // q.denominator
    rem = q - fl
    if rem > F(1, 2) or (rem == F(1, 2) and (fl & 1)):
        fl += 1
    return fl


def f24_mag(b): return b & ((1 << (f24.W - 1)) - 1)


def umax(bits): return (1 << bits) - 1


def z_to_int(z, bits=24):
    """rne(z * (2^bits - 1)) without clamping; FTZ zero / NaN -> 0, +-Inf -> +-2^25"""
    d = fr.decode(z, f24)
    if d[0] in ('zero', 'nan'): return 0
    if d[0] == 'inf': return -(1 << 25) if d[1] else (1 << 25)
    v = rne_int(d[1] * umax(bits))
    return max(-(1 << 25), min(1 << 25, v))


def z_to_int24(z): return z_to_int(z, 24)


def clamp_n(v, bits=24): return 0 if v < 0 else (umax(bits) if v > umax(bits) else v)


def clamp24(v): return clamp_n(v, 24)


def u_to_f24(k, bits=24):
    return fr.encode_real(F(k, umax(bits)), f24) if k else 0


def u24_to_f24(k): return u_to_f24(k, 24)


def poly_offset24(factor, units, dzdx, dzdy, bits=24):
    """o24 = rne(factor * max(|dzdx|, |dzdy|) * (2^bits - 1)) + units, saturated to +-2^25"""
    m = max(f24_mag(dzdx), f24_mag(dzdy))
    v = z_to_int(mul(factor, m), bits) + units
    return max(-(1 << 25), min(1 << 25, v))


def quad_setup_px(t, mu0, mu1, mu2, x, y, o24, bits=24):
    """t: dict of the triangle constants (FP24 bits). Returns the per-pixel outputs (bits / ints); bits = the depth
    attachment's 24 or 16 bits (the d24 value is right-aligned in the 24-bit depth field)."""
    z = mad(mu2, t['dz2'], mad(mu1, t['dz1'], t['z0']))
    fw = mad(mu2, t['q2'], mad(mu1, t['q1'], mul(mu0, t['q0'])))
    r = rcp(fw)
    l1 = mul(mul(mu1, t['q1']), r)
    l2 = mul(mul(mu2, t['q2']), r)
    d24 = clamp_n(z_to_int(z, bits) + o24, bits)
    return dict(z=z, fw=fw, l1=l1, l2=l2, d24=d24, fcz=u_to_f24(d24, bits),
                fcx=enc(F(2 * x + 1, 2)), fcy=enc(F(2 * y + 1, 2)))


def ldv(l1, l2, f0, d1, d2):
    return [mad(l2, d2[c], mad(l1, d1[c], f0[c])) for c in range(4)]


# ---------------------------------------------------------------- integer edge values (A's rasterizer -> B)
# Window positions are integers in 1/16 pixel; a2 = (x1 - x0)(y2 - y0) - (x2 - x0)(y1 - y0) (1/256 pixel^2, up to 34
# bits). Edge k's value at the centre (16 px + 8, 16 py + 8) of pixel (px, py): e1 = (cx - x0)(y2 - y0) - (x2 - x0)(cy - y0),
# e2 = (x1 - x0)(cy - y0) - (cx - x0)(y1 - y0), e0 = a2 - e1 - e2 (mu_k = e_k / a2); per-pixel steps A1 = 16 (y2 - y0),
# B1 = -16 (x2 - x0), A2 = -16 (y1 - y0), B2 = 16 (x1 - x0), A0 = -A1 - A2, B0 = -B1 - B2. Everything A sends is
# right-shifted (floor) by s = max(0, bits(|a2|) - 30), so |a2 >> s| < 2^30 and every edge value of an emitted quad
# (|e| <= |a2| + one step of < 2^21 for a helper pixel) fits a 32-bit word; R = RCP(i2f(a2 >> s)).
def w32(v):
    """a 32-bit two's-complement word as a signed integer (B's adders wrap)"""
    return ((v + (1 << 31)) & 0xFFFFFFFF) - (1 << 31)


def edge_shift(a2i): return max(0, abs(a2i).bit_length() - 30)


def units16(P): return [(int(x * 16), int(y * 16)) for x, y in P]


def tri_edges(P):
    """per triangle: (s, R, [A0, B0, A1, B1, A2, B2]) from the window positions (Fractions on the 1/16 grid)"""
    (x0, y0), (x1, y1), (x2, y2) = units16(P)
    a2 = (x1 - x0) * (y2 - y0) - (x2 - x0) * (y1 - y0)
    s = edge_shift(a2)
    A1, B1, A2, B2 = 16 * (y2 - y0), -16 * (x2 - x0), -16 * (y1 - y0), 16 * (x1 - x0)
    return s, rcp(enc(F(a2 >> s))), [v >> s for v in (-A1 - A2, -B1 - B2, A1, B1, A2, B2)]


def pixel_edges(P, s, px, py):
    """the three edge values (>> s) at the centre of pixel (px, py) — what A's rasterizer sends for a quad's pixel 0"""
    (x0, y0), (x1, y1), (x2, y2) = units16(P)
    a2 = (x1 - x0) * (y2 - y0) - (x2 - x0) * (y1 - y0)
    cx, cy = 16 * px + 8, 16 * py + 8
    e1 = (cx - x0) * (y2 - y0) - (x2 - x0) * (cy - y0)
    e2 = (x1 - x0) * (cy - y0) - (cx - x0) * (y1 - y0)
    return [v >> s for v in (a2 - e1 - e2, e1, e2)]


def band_lines(wid):
    """B's band height for a target of this width (b_render BAND8_MAXW / NLMAX; a_render and a_setup follow the same rule):
    8 rows up to 1024 wide — the band buffer's 4 096 cells hold 8 lines of such a target — else 4. BR_NLINES=8 in the
    environment: the rows above 1024 wide (b_render NLINES, a_render / a_setup NL) — the doubled band buffer's what-if
    (tb_b_render -Defines TB_NL8)"""
    return 8 if wid <= 1024 else int(os.environ.get('BR_NLINES', '4'))


def ref_edges(P, s):
    """the three edge values at the centre of pixel (0, 0) — (8, 8) in 1/16 pixel — >> s,
    as 32-bit words: the TRI item's w22..w24. B forms a quad's pixel-0 values from them and the TRI's steps modulo 2^32
    (quad_edges_ref)."""
    return [v & 0xFFFFFFFF for v in pixel_edges(P, s, 0, 0)]


def quad_edges_ref(er, AB, qx, qy):
    """B: the quad's pixel-0 edge values from the reference words er and the shifted steps AB, 32-bit wrapping arithmetic
    (equal to pixel_edges for s = 0; for s > 0 the steps' dropped fraction bits make it differ by a few units)"""
    return [w32(er[k] + AB[2 * k] * (2 * qx) + AB[2 * k + 1] * (2 * qy)) for k in range(3)]


def edge_steps_raw(P):
    """the per-pixel edge steps (A0, B0, A1, B1, A2, B2) unshifted, and the signed area a2 (1/256 px^2)"""
    (x0, y0), (x1, y1), (x2, y2) = units16(P)
    a2 = (x1 - x0) * (y2 - y0) - (x2 - x0) * (y1 - y0)
    A1, B1, A2, B2 = 16 * (y2 - y0), -16 * (x2 - x0), -16 * (y1 - y0), 16 * (x1 - x0)
    return [-A1 - A2, -B1 - B2, A1, B1, A2, B2], a2


def pixel_covered(e, AB, front):
    """ES 2.0 §3.5.1: the pixel centre is inside iff every edge value, normalised to the triangle's
    orientation (e * sign(a2)), is > 0, or == 0 on an edge that owns its boundary: the one whose normalised x step is
    > 0, or 0 with the y step > 0 (two triangles sharing an edge have opposite normalised steps along it, so exactly
    one of them produces a centre on it). e, AB unshifted (exact)."""
    for k in range(3):
        ek, Ak, Bk = (e[k], AB[2 * k], AB[2 * k + 1]) if front else (-e[k], -AB[2 * k], -AB[2 * k + 1])
        if ek < 0: return False
        if ek == 0 and not (Ak > 0 or (Ak == 0 and Bk > 0)): return False
    return True


def quad_cover(P, qx, qy, wid, hgt):
    """the coverage bits of quad (qx, qy): pixel p = (2 qx + (p & 1), 2 qy + (p >> 1)) inside the target and covered"""
    AB, a2 = edge_steps_raw(P)
    front = a2 > 0
    cov = 0
    for p in range(4):
        px, py = 2 * qx + (p & 1), 2 * qy + (p >> 1)
        if px < wid and py < hgt and pixel_covered(pixel_edges(P, 0, px, py), AB, front): cov |= 1 << p
    return cov


def quad_pixel_edges(t, e, p):
    """B: pixel p's edge values from the quad's pixel-0 values e and the triangle's steps t['AB'] (32-bit adds)"""
    return [w32(e[k] + t['AB'][2 * k] * (p & 1) + t['AB'][2 * k + 1] * (p >> 1)) for k in range(3)]


def mu_b(e, R): return mul(enc(F(e)), R)


def quad_setup_e(t, e, p, x, y, o24, bits=24):
    """quad_setup from the quad's pixel-0 edge values e for pixel p at (x, y): adds the mus to quad_setup_px's result"""
    ep = quad_pixel_edges(t, e, p)
    mu = [mu_b(v, t['R']) for v in ep]
    o = quad_setup_px(t, mu[0], mu[1], mu[2], x, y, o24, bits)
    o['mu'] = mu; o['ep'] = ep
    return o


# ---------------------------------------------------------------- random triangles and pixels (vector generation)
def rand_tri(rng):
    """a screen triangle: window positions (Q.4 sub-pixel), window z in [0, 1], w_c > 0, a few varyings"""
    W, H = 2048, 2048
    while True:
        P = [(F(rng.randrange(0, W * 16), 16), F(rng.randrange(0, H * 16), 16)) for _ in range(3)]
        a2 = (P[1][0] - P[0][0]) * (P[2][1] - P[0][1]) - (P[2][0] - P[0][0]) * (P[1][1] - P[0][1])
        if a2 != 0: break
    kind = rng.randrange(6)
    if kind == 0: zs = [rng.random()] * 3                          # constant depth
    elif kind == 1: zs = [rng.choice([0.0, 1.0]) for _ in range(3)]  # depth at the range ends
    else: zs = [rng.random() for _ in range(3)]
    ws = [rng.choice([1.0, rng.uniform(0.05, 50.0)]) for _ in range(3)]
    if rng.randrange(5) == 0: ws = [ws[0]] * 3                      # affine (orthographic)
    z = [enc(F(v)) for v in zs]
    q = [rcp(enc(F(v))) for v in ws]                                 # q = 1/w as the VS side would compute it
    return P, a2, z, q, zs, ws


def sub(a, b): return fr.fadd(a, b ^ (1 << (f24.W - 1)), f24)


def tri_consts(z, q, dzdx, dzdy, P=None):
    """the record's FP24 constants as A's setup sends them (differences rounded once); with the window positions P
    also the edge R and steps (R = 0, steps 0 otherwise: callers that feed mus directly)"""
    s, R, AB = tri_edges(P) if P is not None else (0, 0, [0] * 6)
    return dict(z0=z[0], dz1=sub(z[1], z[0]), dz2=sub(z[2], z[0]), q0=q[0], q1=q[1], q2=q[2],
                dzdx=dzdx, dzdy=dzdy, R=R, AB=AB, s=s)


def dz_grad(z, P, a2):
    """dz/dx, dz/dy as board A's setup computes them (a_setup.sv; tools/a_setup_ref.py): the window deltas in 1/16 pixel
    converted to FP24 (exact), the area in 1/256 pixel^2 as FP24 with its exponent lowered by 4 (= 16 * area in pixel^2),
    dz/dx = (dz1 * fdy2 - dz2 * fdy1) * rcp(area / 16), dz/dy = (dz2 * fdx1 - dz1 * fdx2) * rcp(area / 16); dz1 = z1 - z0,
    dz2 = z2 - z0 (FP24, rounded once). P = window positions as Fractions on the 1/16 grid, a2 = the signed area in
    pixel^2 (Fraction). Only the polygon offset uses these (ES 2.0 allows an approximation)."""
    (x0, y0), (x1, y1), (x2, y2) = P
    q = lambda v: int(v * 16)
    fdx1, fdy1, fdx2, fdy2 = (enc(F(q(x1) - q(x0))), enc(F(q(y1) - q(y0))), enc(F(q(x2) - q(x0))), enc(F(q(y2) - q(y0))))
    dz1, dz2 = sub(z[1], z[0]), sub(z[2], z[0])
    r = rcp(enc(a2 * 16))
    return mul(sub(mul(dz1, fdy2), mul(dz2, fdy1)), r), mul(sub(mul(dz2, fdx1), mul(dz1, fdx2)), r)


TRI_KEYS = ('z0', 'dz1', 'dz2', 'q0', 'q1', 'q2', 'dzdx', 'dzdy')


def gen_qs(seed, ntri, d16=0):
    rng = random.Random(seed)
    bits = 16 if d16 else 24
    lines = [f"D {d16:x}"]
    maxerr_z = maxerr_v = maxerr_w = 0.0
    npx = 0
    for ti in range(ntri):
        P, a2, z, q, zs, ws = rand_tri(rng)
        # dz/dx, dz/dy of the screen plane as A's setup computes them (dz_grad)
        (x0, y0), (x1, y1), (x2, y2) = P
        dzdx, dzdy = dz_grad(z, P, a2)
        t = tri_consts(z, q, dzdx, dzdy, P)
        pk = rng.randrange(4)
        factor = 0 if pk == 0 else enc(F(rng.uniform(-4.0, 4.0)))
        units = 0 if pk == 0 else rng.randrange(-300, 300)
        po_en = 1 if pk != 0 else 0
        o24 = poly_offset24(factor, units, dzdx, dzdy, bits) if po_en else 0
        # a vec4 varying with random vertex values: f0, D1, D2 for the interpolation check
        fv = [[enc(F(rng.uniform(-8, 8))) for _ in range(4)] for _ in range(3)]
        if rng.randrange(4) == 0: fv = [fv[0]] * 3                 # constant varying
        f0 = fv[0]
        D1 = [sub(fv[1][c], fv[0][c]) for c in range(4)]
        D2 = [sub(fv[2][c], fv[0][c]) for c in range(4)]
        lines.append("T " + " ".join(f"{t[k]:06x}" for k in TRI_KEYS)
                     + f" {po_en:x} {factor:06x} {units & 0xFFFFFF:06x} {o24 & 0x7FFFFFF:07x}"
                     + f" {t['R']:06x} " + " ".join(f"{v & 0xFFFFFFFF:08x}" for v in t['AB']))
        for _ in range(rng.randrange(1, 9)):
            # a pixel centre: mostly inside, some outside (helper pixels), some exactly on a vertex
            k = rng.randrange(8)
            if k == 0:
                v = rng.randrange(3); px, py = P[v][0] - F(1, 2), P[v][1] - F(1, 2)
                px, py = F(int(px)), F(int(py))
            else:
                b = [rng.random() for _ in range(3)]; s = sum(b)
                cx = sum(F(b[i] / s) * P[i][0] for i in range(3)); cy = sum(F(b[i] / s) * P[i][1] for i in range(3))
                px, py = F(int(cx)), F(int(cy))
                if k == 1: px += rng.randrange(-3, 4); py += rng.randrange(-3, 4)
            px = max(F(0), min(F(4095), px)); py = max(F(0), min(F(4095), py))
            cxr, cyr = px + F(1, 2), py + F(1, 2)
            e1 = (cxr - x0) * (y2 - y0) - (x2 - x0) * (cyr - y0)       # barycentric areas of vertices 1 and 2
            e2 = (x1 - x0) * (cyr - y0) - (cxr - x0) * (y1 - y0)
            mu1r, mu2r = e1 / a2, e2 / a2
            # the pixel as pixel p of a quad: A sends the quad's pixel-0 edge values, B steps to pixel p
            p = rng.randrange(4)
            eq = pixel_edges(P, t['s'], int(px) - (p & 1), int(py) - (p >> 1))
            o = quad_setup_e(t, eq, p, int(px), int(py), o24, bits)
            mu0, mu1, mu2 = o['mu']
            assert t['s'] != 0 or o['ep'] == pixel_edges(P, 0, int(px), int(py)), "edge steps"
            lines.append(f"P {eq[0] & 0xFFFFFFFF:08x} {eq[1] & 0xFFFFFFFF:08x} {eq[2] & 0xFFFFFFFF:08x} {p:x} "
                         f"{int(px):03x} {int(py):03x} {o['z']:06x} {o['fw']:06x} "
                         f"{o['l1']:06x} {o['l2']:06x} {o['d24']:06x} {o['fcz']:06x} {o['fcx']:06x} {o['fcy']:06x} "
                         f"{mu1:06x} {mu2:06x}")
            npx += 1
            # exactness of z and of an interpolated varying against the exact rational values (for the inside pixels;
            # exact = eq. 3.4 with the barycentrics B forms, i.e. after their FP24 rounding)
            if 0 <= mu1r and 0 <= mu2r and mu1r + mu2r <= 1:
                m0, m1, m2 = val(mu0), val(mu1), val(mu2)
                zx = val(z[0]) + m1 * (val(z[1]) - val(z[0])) + m2 * (val(z[2]) - val(z[0]))
                if zx != 0:
                    maxerr_z = max(maxerr_z, float(abs(val(o['z']) - zx) * (1 << 24)))
                qv = [val(v) for v in q]
                den = m0 * qv[0] + m1 * qv[1] + m2 * qv[2]
                maxerr_w = max(maxerr_w, fr.ulp_err(o['fw'], den, f24))
                v = ldv(o['l1'], o['l2'], f0, D1, D2)
                for c in range(4):
                    fx = (m0 * qv[0] * val(fv[0][c]) + m1 * qv[1] * val(fv[1][c]) + m2 * qv[2] * val(fv[2][c])) / den
                    mag = max(abs(val(fv[i][c])) for i in range(3))
                    if mag != 0:
                        # error relative to the largest vertex value's ULP (the delta form's natural scale)
                        e = abs(val(v[c]) - fx) / (F(2) ** (mag.numerator.bit_length() - mag.denominator.bit_length()
                                                            - f24.M))
                        maxerr_v = max(maxerr_v, float(e))
                    if fv[0][c] == fv[1][c] == fv[2][c]:
                        assert v[c] == fv[0][c], "constant varying not exact"
    with open(os.path.join(GEN, "qs_vectors.hex"), "w") as fh:
        fh.write("\n".join(lines) + "\nE\n")
    print(f"qs: {ntri} triangles, {npx} pixels, depth {bits} bits -> tools/gen/qs_vectors.hex")
    print(f"    max |z error| {maxerr_z:.1f} x 2^-24, max 1/w error {maxerr_w:.2f} ULP, max varying error "
          f"{maxerr_v:.2f} ULP of the largest vertex value (constant varyings exact)")


# ---------------------------------------------------------------- ROP (b_rop): colour in n.4 fixed point per channel
# ES 2.0 §2.1.2 / §4.1.6 / §4.1.7: source and destination are fixed point (m >= framebuffer bits) for blending, the
# blend acts as if in floating point, its result is converted by rounding to nearest, then dithered (one of the two
# neighbours, depending only on the value and x, y) or truncated. Here a colour component of a channel of n bits is
# value * (2^n - 1) * 16 (n integer + 4 fraction bits of the channel's step; c16 = 4080 units for RGBA8), factors are
# k / ONE (k = 0..ONE). Render-target formats (fmt as tmu24): 0 RGBA8, 1 RGBA4444, 2 RGBA5551, 3 RGB565 (alpha: no
# bits, destination alpha 1.0).
C16 = 4080
CHN = {0: (8, 8, 8, 8), 1: (4, 4, 4, 4), 2: (5, 5, 5, 1), 3: (5, 6, 5, 0)}      # channel bits {R, G, B, A}
FMT_BPP = {0: 4, 1: 2, 2: 2, 3: 2}


def one_of(n): return 16 * ((1 << n) - 1)


def f24_cn(f, n):
    """clamp(f, 0, 1) * ONE rounded to nearest, ties up (as b_render's c16_b2: truncate + round bit); NaN -> 0.
    Ties exist (f = 3/32 -> 382.5 for n = 8); §2.1.2 allows either neighbour."""
    one = one_of(n)
    d = fr.decode(f, f24)
    if d[0] in ('zero', 'nan'): return 0
    if d[0] == 'inf': return 0 if d[1] else one
    v = d[1]
    if v <= 0: return 0
    if v >= 1: return one
    q = v * one + F(1, 2)
    return q.numerator // q.denominator


def f24_c16(f): return f24_cn(f, 8)


def f24_u24(f, bits=24):
    """gl_FragDepth: clamp(f, 0, 1) * (2^bits - 1) rounded to nearest even"""
    return clamp_n(z_to_int(f, bits), bits)


def unpack_px(v, fmt):
    """a stored pixel (32 or 16 bits) -> the band cell's colour word {R, G, B, A} of n-bit values in bytes"""
    if fmt == 0: return v
    if fmt == 1: return ((v >> 12) & 15) << 24 | ((v >> 8) & 15) << 16 | ((v >> 4) & 15) << 8 | (v & 15)
    if fmt == 2: return ((v >> 11) & 31) << 24 | ((v >> 6) & 31) << 16 | ((v >> 1) & 31) << 8 | (v & 1)
    return ((v >> 11) & 31) << 24 | ((v >> 5) & 63) << 16 | (v & 31) << 8


def pack_px(c, fmt):
    """the cell's colour word -> the stored pixel"""
    r, g, b, a = (c >> 24) & 255, (c >> 16) & 255, (c >> 8) & 255, c & 255
    if fmt == 0: return c
    if fmt == 1: return r << 12 | g << 8 | b << 4 | a
    if fmt == 2: return r << 11 | g << 6 | b << 1 | a
    return r << 11 | g << 5 | b


def bayer(x, y):
    return [0, 8, 2, 10, 12, 4, 14, 6, 3, 11, 1, 9, 15, 7, 13, 5][((y & 3) << 2) | (x & 3)]


def dither8(c16, x, y, en):
    """c16 -> 8 bits: floor((c16 + t) / 16), t = the 4x4 Bayer threshold (0..15) or 0 (dither off = truncation)"""
    return (c16 + (bayer(x, y) if en else 0)) >> 4


def div4080(x):
    """round(x / 4080), x >= 0, half up — as the RTL: floor(floor((x + 2040) / 16) / 255) with the /255 by shifts"""
    w = (x + 2040) >> 4
    return (w + 1 + (w >> 8)) >> 8 if False else w // 255


def blend_fx(s, d, one, cc, fsrc, fdst, op, a):
    """one channel of n bits (one = 16 (2^n - 1)): s = source value of the channel in its units, d = the stored n-bit
    destination value, a = (source alpha in the channel's units, destination alpha as a factor in the channel's units);
    cc = (constant channel, constant alpha in the channel's units); factor / op codes as pixel_blend_b."""
    sa, da = a
    ccol, calp = cc
    d16 = 16 * d
    def fac(code, is_dst):
        return [0, one, s, one - s, d16, one - d16, sa, one - sa, da, one - da,
                ccol, one - ccol, calp, one - calp, min(sa, one - da), 0][code] if not is_dst or code != 14 else 0
    if op == 3: return min(s, d16)
    if op == 4: return max(s, d16)
    x1, x2 = s * fac(fsrc, False), d16 * fac(fdst, True)
    x = x1 + x2 if op == 0 else (x1 - x2 if op == 1 else x2 - x1)
    if x <= 0: return 0
    return min(one, (x + one // 2) // one)


def blend_c16(s, d8, cc, fsrc, fdst, op, a=None):
    """RGBA8 channel (c16 units); a = (source alpha c16, destination alpha byte)"""
    return blend_fx(s, d8, C16, cc, fsrc, fdst, op, (a[0], 16 * a[1]))


def dst_alpha_fac(da, n_a, n_c):
    """the stored alpha (n_a bits) as a factor of a channel of n_c bits, in that channel's units: 16 dA when the widths
    agree, 0 / ONE for a 1-bit alpha, ONE (1.0) when the target has no alpha"""
    if n_a == 0: return one_of(n_c)
    if n_a == n_c: return 16 * da
    return one_of(n_c) if da else 0


def alpha_fac_code(code):
    """the alpha column of table 4.2: colour factors use their alpha; SRC_ALPHA_SATURATE -> ONE"""
    return {2: 6, 3: 7, 4: 8, 5: 9, 10: 12, 11: 13, 14: 1}.get(code, code)


DIV_K = 33
DIV_M = -(-(1 << DIV_K) // 255)          # ceil(2^33 / 255)


def check_div():
    """the RTL's exact divisions: floor(w / (2^n - 1)) = (w * ceil(2^33 / (2^n - 1))) >> 33 for every w = (x + ONE/2) >> 4,
    x <= 2 ONE^2, of every channel width n (pixel_blend_b divm)"""
    for n in (1, 4, 5, 6, 8):
        d = (1 << n) - 1; one = one_of(n); m = -(-(1 << DIV_K) // d)
        wmax = (2 * one * one + one // 2) >> 4
        bad = sum(1 for w in range(wmax + 1) if (w * m) >> DIV_K != w // d)
        print(f"n = {n}: floor(w / {d}) = (w * {m}) >> {DIV_K} for w <= {wmax}: {bad} mismatches")


def write_mem(path, words):
    with open(path, "w") as fh:
        fh.write("\n".join(f"{w:08x}" for w in words) + "\n")


LOW_L1, LOW_L2, LOW_V0 = 12, 13, 16          # reference-core varin slots: l1, l2 broadcast; f0 / D1 / D2 of varying i


def lower_ldv(lines):
    """the program for a plain (INTERP = 0) core that computes what the INTERP core computes: each LDV of a user varying
    i becomes its two MADs over varin slots (l1 / l2 per pixel on all components, f0 / D1 / D2 on all pixels)"""
    import re
    out = []
    for ln in lines:
        code = ln.split('//')[0].strip()
        t = re.split(r'[\s,]+', code) if code else []
        if t and t[0].upper() == 'LDV' and int(t[2]) < 8:
            rd, i = t[1], int(t[2])
            out += [f"LDV r28, {LOW_L1}", f"LDV r31, {LOW_V0 + 3 * i + 1}", f"LDV r30, {LOW_V0 + 3 * i}",
                    f"MAD {rd}, r28, r31, r30", f"LDV r29, {LOW_L2}", f"LDV r30, {LOW_V0 + 3 * i + 2}",
                    f"MAD {rd}, r29, r30, {rd}"]
        else:
            out.append(ln)
    return out


def gen_fsi(prog):
    import shader_ref as sr
    name = os.path.splitext(os.path.basename(prog))[0]
    lines = open(prog).read().split('\n')
    w, _ = sr.assemble(lines)
    wl, _ = sr.assemble(lower_ldv(lines))
    write_mem(os.path.join(GEN, f"{name}_fsi_iram.mem"), w)
    write_mem(os.path.join(GEN, f"{name}_fsil_iram.mem"), wl)
    print(f"fsi: {name}: {len(w)} words (INTERP core), {len(wl)} words lowered -> tools/gen/{name}_fsi[l]_iram.mem")


# ---------------------------------------------------------------- frame model + generator (tb_b_render)
# The render stream, DDR contents (buckets, programs, the render target) and the expected image of
# one frame: random triangles of several draws are rasterized here (A's job: coverage at pixel centres, quads of the
# bands, the barycentrics mu0..mu2 per pixel), and rendered with the bit-exact models: quad setup, LDV interpolation,
# shader_ref's SIMT core at FP24, the b_rop rules.
B_REC_BASE, B_DS_BASE, B_PROG_BASE, B_RT_BASE = 0x0, 0x0200_0000, 0x0400_0000, 0x0800_0000   # word addresses
B_TEX_BASE = 0x0900_0000


class Draw:
    pass


def rop_frag(st, band, x, y, l, c16, z, front, fmt=0, sa=None):
    """one fragment through the b_rop rules; band = (colour, depth, stencil) rows of the band; st = draw state dict;
    c16 = the source colour {R, G, B, A} in the target's channel units (13 bits each); sa = (source alpha in G's
    units, in R / B's units), defaults to the alpha channel's value (RGBA8); fmt = the render-target format"""
    if st['scissor'] and not (st['sx0'] <= x < st['sx1'] and st['sy0'] <= y < st['sy1']): return
    bc, bz, bs = band
    f = 1 if front else 0
    def cmp(fn, a, b): return [False, a < b, a == b, a <= b, a > b, a != b, a >= b, True][fn]
    stv = bs[l][x]
    sp = (not st['stencil']) or cmp(st['sfn'][f], st['sref'][f] & st['svm'][f], stv & st['svm'][f])
    dp = (not st['depth_test']) or cmp(st['depth_func'], z, bz[l][x])
    if st['stencil']:
        o = st['odp'][f] if (sp and dp) else (st['odf'][f] if sp else st['osf'][f])
        r = st['sref'][f]
        n = [stv, 0, r, min(stv + 1, 255), max(stv - 1, 0), (~stv) & 255, (stv + 1) & 255, (stv - 1) & 255][o]
        bs[l][x] = (n & st['swm'][f]) | (stv & ~st['swm'][f] & 255)
    if sp and dp:
        if st['depth_mask']: bz[l][x] = z
        d = bc[l][x]
        nb = CHN[fmt]
        if sa is None: sa = (c16 & 0x1FFF, c16 & 0x1FFF)
        out = 0
        for c in range(4):
            s = (c16 >> ((3 - c) * 13)) & 0x1FFF
            dch = (d >> ((3 - c) * 8)) & 0xFF
            if not st['blend']: v = s
            else:
                cc = st['bcc']
                ccol = (cc >> ((3 - c) * 13)) & 0x1FFF
                da = dst_alpha_fac(d & 0xFF, nb[3], nb[c])
                if c < 3:
                    sac = sa[0] if c == 1 else sa[1]
                    cac = (st['bca'] >> 13) & 0x1FFF if c == 1 else st['bca'] & 0x1FFF
                    v = blend_fx(s, dch, one_of(nb[c]), (ccol, cac), st['bsf'], st['bdf'], st['bop'], (sac, da))
                else:
                    v = blend_fx(s, dch, one_of(nb[c]), (ccol, ccol), alpha_fac_code(st['bsfa']),
                                 alpha_fac_code(st['bdfa']), st['bopa'], (s, da))
            b8 = dither8(v, x, y, st['dither']) if (st['cmask'] >> (3 - c)) & 1 else dch
            out |= b8 << ((3 - c) * 8)
        bc[l][x] = out


def pack_state(st, prog_addr, plen, nuni):
    w = [0] * 16
    w[0] = (st['depth_test'] | st['depth_mask'] << 1 | st['depth_func'] << 2 | st['stencil'] << 5 | st['blend'] << 6
            | st['dither'] << 7 | st['scissor'] << 8 | st['po'] << 9 | st['cmask'] << 16)
    for k, f in ((1, 1), (2, 0)):
        w[k] = st['sref'][f] | st['svm'][f] << 8 | st['swm'][f] << 16 | st['sfn'][f] << 24
    w[3] = (st['osf'][1] | st['odf'][1] << 3 | st['odp'][1] << 6 | st['osf'][0] << 16 | st['odf'][0] << 19
            | st['odp'][0] << 22)
    w[4] = st['bsf'] | st['bdf'] << 4 | st['bsfa'] << 8 | st['bdfa'] << 12 | st['bop'] << 16 | st['bopa'] << 20
    cc = st['bcc']; R, G, B, A = [(cc >> ((3 - c) * 13)) & 0x1FFF for c in range(4)]
    w[5] = R | G << 13; w[6] = B | A << 13
    w[7] = st['sx0'] | st['sy0'] << 16; w[8] = st['sx1'] | st['sy1'] << 16
    w[9] = st['pofac']; w[10] = st['pounits'] & 0xFFFFFF
    w[11] = prog_addr; w[12] = plen | nuni << 16
    w[13] = st.get('bca', 0)                     # the constant alpha {in G's units [25:13], in R / B's units [12:0]}
    return w


def words_to_bursts(words):
    words = words + [0] * (-len(words) % 8)
    return [sum(words[b * 8 + k] << (32 * k) for k in range(8)) for b in range(len(words) // 8)]


B_DSI_BASE = 0x0A00_0000                       # the depth-stencil image (word address)


def tex_skip_ok(words):
    """b_render's program scan, written again from its rules: may the
    uncovered pixels of this program's quads go unsampled (zeros instead of texels)? A register component is MARKED
    when its value may differ in an uncovered pixel: written by a TEX, by an operation reading a marked component, or
    by anything under an IF whose predicate is marked; an unconditional write from unmarked sources clears the mark.
    The program is refused when a marked value can reach another pixel — DDX / DDY of it, a TEX's coordinates — or
    when it holds what the scan does not follow (loops, BRA, an opcode the fragment core does not have)."""
    import isa64
    N0 = {0, 2, 16, 17, 63, 20, 22, 23, 24, 25, 26, 27, 28, 29}                 # no register source
    N1 = {1, 19, 21, 11, 12, 13, 14, 30, 31, 32, 33, 34, 40, 41, 42, 43, 44, 45, 55, 56}   # one
    N3 = {6, 46}                                                                # three (the others: two)
    WR = set(range(1, 15)) | {16, 17} | set(range(30, 51)) | {52, 53, 54, 55, 56}          # write rd.wm
    NG = {18, 20, 22, 23, 27, 28, 51} | set(range(57, 63))                      # not followed
    t = [[False] * 4 for _ in range(32)]; ifd = 0; ct = False; tp = False
    for w in words:
        d = isa64.decode(w); op = d['op']; wm = d['wm']
        if op > 63 or op in NG: return False
        if op == 9: rs = 0b0111                                                 # DP3 / DP4: positions 0..2 / 0..3
        elif op == 10: rs = 0b1111
        elif op == 34: rs = ((w >> 6) & 0xF) or 0xF                             # TEX: its coordinate set
        elif op == 15: rs = (wm & -wm) or 1                                     # SETP: one micro-op, wm's first component
        else: rs = wm
        def marked(k):
            return any((rs >> c) & 1 and t[d[f'rs{k}']][(d[f'swz{k}'] >> (2 * c)) & 3] for c in range(4))
        s0 = op not in N0 and marked(0)
        src = s0 or (op not in N0 and op not in N1 and marked(1)) or (op in N3 and marked(2))
        if op in (32, 33, 34) and s0: return False
        if (op == 24 and ifd == 15) or (op == 26 and ifd == 0): return False
        if op in WR:
            for c in range(4):
                if (wm >> c) & 1: t[d['rd']][c] = src or op == 34 or ct or (ifd != 0 and t[d['rd']][c])
        if op == 15: tp = src or ct or (ifd != 0 and tp)
        if op == 24:
            ifd += 1
            if tp: ct = True
        if op == 26:
            if ifd == 1: ct = False
            ifd -= 1
    return True


def gen_frame(seed, ntri=20, WID=64, HGT=32, NL=None):
    if NL is None: NL = band_lines(WID)                             # the band height follows the target's width
    import isa64                               # the 64-bit ISA: programs as 64-bit instructions, the scalar warp simulator
    rng = random.Random(seed)
    import tmu_ref as tr
    units = isa64.Units(exact_rcp=True)
    # the frame's render target: format, depth bits, depth-stencil image (from the seed, or forced by the environment)
    cfg = random.Random(seed * 7919 + 13)
    fmt = int(os.environ.get('BR_FMT', cfg.choice([0, 0, 1, 2, 3])))
    d16 = int(os.environ.get('BR_D16', cfg.choice([0, 0, 1])))
    ds = int(os.environ.get('BR_DS', cfg.choice([0, 1, 1])))
    dbits = 16 if d16 else 24
    nb = CHN[fmt]; bpp = FMT_BPP[fmt]; ppw = 4 // bpp          # pixels per 32-bit word
    assert (WID * bpp) % 32 == 0, "the target's pitch must be a multiple of 32 bytes"
    ddr = {}                                   # burst word address -> 256-bit
    # textures: sampler 0 a 2D texture with a mip chain, sampler 1 a cube map (tmu_ref's random descriptors / texels)
    texmem = {}
    # BR_TEX2D=1: sampler 1 a 2D texture too; BR_TEXCFG=w,h,levels,filt,fmt[;w,h,levels,filt,fmt]: the samplers' geometry
    # forced (REPEAT, complete) and the second texture placed after the first (the texture-rate benches)
    tex = [tr.make_desc(rng, B_TEX_BASE, cube=False),
           tr.make_desc(rng, B_TEX_BASE + 0x10000, cube=(os.environ.get('BR_TEX2D', '0') != '1'))]
    if os.environ.get('BR_TEXCFG'):
        specs = os.environ['BR_TEXCFG'].split(';')
        for k, d in enumerate(tex):
            w, h, lv, filt, fm = (int(x, 0) for x in specs[min(k, len(specs) - 1)].split(','))
            d.update(w=w, h=h, d=1, levels=lv, filt=filt, fmt=fm, wrap_s=0, wrap_t=0, wrap_r=0, cube=0, vol=0, incomplete=0)
        tex[1]['base'] = B_TEX_BASE + ((tr.level_offsets(tex[0])[1] + 0xFFFF) & ~0xFFFF)
    for d in tex:
        d['incomplete'] = 0
        tr.make_texture(rng, texmem, d['base'], d)
    for a, v in texmem.items():
        b = a & ~7
        ddr[b] = ddr.get(b, 0) | (v << (32 * (a & 7)))
    def tex_fn(samp, mode, coords):
        d = tex[samp] if samp < 2 else dict(tex[0], incomplete=1)
        return tr.sample(d, texmem, dict(samp=samp, mode=mode, c=[tuple(coords[p * 4 + c] for c in range(4))
                                                                   for p in range(4)]), f24)
    def sampler_bits(d):
        return (d['base'] | d['w'] << 32 | d['h'] << 48 | d['fmt'] << 64 | d['levels'] << 66 | d['filt'] << 70
                | d['wrap_s'] << 74 | d['wrap_t'] << 76 | d['incomplete'] << 79 | d['cube'] << 80)
    progs = {}
    # the texture-study programs join the list only when BR_PROG0 names one of them: the draw count follows the
    # list and the triangles' random state follows the draws, so the standard frames (seed 1 = 299 quads) must not change
    # and the study frames must stay the ones measured (all four appended)
    prog0 = os.environ.get('BR_PROG0', 'b_fs_two')
    study = ('b_fs_tex1', 'b_fs_tex2', 'b_fs_tex1m', 'b_fs_tex2p' if prog0 == 'b_fs_tex2p' else 'b_fs_tex2m')   # the packed form in b_fs_tex2m's place: the same draws
    # BR_PROG0=b_fs_alu16: fb_bench's alu_16 shape — two uniforms and 16 chained vec4 multiply-adds — as draw 0
    alu = ('b_fs_alu16',) if prog0 == 'b_fs_alu16' else ()
    # BR_SKIPTEST=1: the sample-skip scan's programs join the list — three it must
    # refuse (a dependent read, DDX / DDY of a texel, coordinates changed under a texel's IF), three it must allow (an
    # alpha test, a texel's register written again, a part of it written again); br_skip.hex = the verdict per program
    skt = ('b_fs_sk_dep', 'b_fs_sk_ddx', 'b_fs_sk_at', 'b_fs_sk_ifc', 'b_fs_sk_reuse', 'b_fs_sk_part') if os.environ.get('BR_SKIPTEST') == '1' else ()
    # BR_PROG0=b_fs_oa_*: OpenArena's fixed-function programs and their constant-colour
    # variants (tests/b_fs_oa_*.sasm), as draw 0 of a BR_LAYERS frame
    oa = (prog0,) if prog0.startswith('b_fs_oa_') else ()
    for name in ('b_fs_flat', 'b_fs_two', 'b_fs_disc', 'b_fs_depth', 'b_fs_point', 'b_fs_tex', 'b_fs_cube') + (study if prog0 in study else ()) + alu + skt + oa:
        lines = open(os.path.join(os.path.dirname(__file__), '..', 'tests', name + '.sasm')).read().split('\n')
        words, _ = isa64.assemble(lines)
        mask = 0
        for w in words:
            if (w >> 57) == 17 and (w & 0xFF) < 8: mask |= 1 << (w & 0xF)      # LDV of a user varying
        progs[name] = (words, mask)
    def rstate(i):
        st = dict(depth_test=rng.choice([0, 1, 1]), depth_mask=rng.choice([0, 1, 1]), depth_func=rng.choice([1, 1, 3, 4, 6, 7, 5, 2]),
                  stencil=rng.choice([0, 0, 1]), blend=rng.choice([0, 1]), dither=rng.choice([0, 1]),
                  scissor=rng.choice([0, 0, 0, 1]), po=rng.choice([0, 0, 1]),
                  cmask=rng.choice([15, 15, 15, rng.randrange(16)]),
                  sref=[rng.randrange(4), rng.randrange(4)], svm=[255, rng.randrange(256)], swm=[255, rng.randrange(256)],
                  sfn=[rng.randrange(8), rng.randrange(8)], osf=[rng.randrange(8) for _ in range(2)],
                  odf=[rng.randrange(8) for _ in range(2)], odp=[rng.randrange(8) for _ in range(2)],
                  bsf=rng.randrange(15), bdf=rng.randrange(14), bsfa=rng.randrange(15), bdfa=rng.randrange(14),
                  bop=rng.randrange(5), bopa=rng.randrange(5),
                  sx0=rng.randrange(20), sy0=rng.randrange(12), pofac=enc(F(rng.uniform(-2, 2))),
                  pounits=rng.randrange(-200, 200))
        # the blend constant: a float per channel, converted by the driver into the target's channel units; the
        # constant alpha also into the units of the colour channels (word 13)
        cf = [rng.choice([0.0, 1.0, rng.random()]) for _ in range(4)]
        cu = [round(cf[c] * one_of(nb[c])) for c in range(4)]
        st['bcc'] = sum(cu[c] << (13 * (3 - c)) for c in range(4))
        st['bca'] = round(cf[3] * one_of(nb[0])) | round(cf[3] * one_of(nb[1])) << 13
        st['sx1'] = st['sx0'] + rng.randrange(8, 50); st['sy1'] = st['sy0'] + rng.randrange(4, 24)
        if i == 0: st.update(depth_test=1, depth_mask=1, depth_func=1, stencil=0, blend=0, scissor=0, po=0, cmask=15)
        return st
    names = list(progs)
    draws = []
    for i in range(len(names)):
        d = Draw(); d.id = i * 3 + 1; d.st = rstate(i); d.prog = names[i % len(names)] if i else prog0
        d.words, d.mask = progs[d.prog]
        d.w32 = [x for w in d.words for x in (w & 0xFFFFFFFF, w >> 32)]    # the buffer holds 32-bit words, low first
        # every uniform the program reads (LDP index), as a GL driver sends them; sometimes a few more
        nu = 1 + max([w & 0xFF for w in d.words if (w >> 57) == 16] + [0])
        d.uni = [[enc(F(rng.uniform(0.0, 1.5))) for _ in range(4)] for _ in range(nu + rng.randrange(0, 3))]
        d.paddr = B_PROG_BASE + i * 4096
        for k, b in enumerate(words_to_bursts(list(d.w32))): ddr[d.paddr + 8 * k] = b
        sw = pack_state(d.st, d.paddr, len(d.words), len(d.uni))
        bucket = words_to_bursts(sw) + [sampler_bits(tex[0]) | sampler_bits(tex[1]) << 128, 0, 0, 0]   # state, samplers
        for k in range(0, len(d.uni), 2):
            u = d.uni[k:k + 2] + [[0] * 4] * (2 - len(d.uni[k:k + 2]))
            bucket.append(sum(u[h][c] << (h * 128 + 32 * c) for h in range(2) for c in range(4)))
        for k, b in enumerate(bucket): ddr[B_DS_BASE + d.id * 512 + 8 * k] = b
        d.o24 = None
        draws.append(d)
    # triangles: window positions in 1/16 pixel, z in [0, 1], w in [0.3, 30], vertex varyings in [0, 1.2)
    # BR_LAYERS=n (overdraw benches): instead, n layers of two triangles covering the target, all of draw 0
    # (LESS, depth writes), slightly sloped in depth and never crossing; BR_ORDER=front (default: front to back, every layer
    # after the first hidden) or back (back to front: none hidden)
    layers = int(os.environ.get('BR_LAYERS', '0'))
    # BR_GRID=a: instead, fb_bench's tri_<a>px scene — a grid of cells
    # of 2a pixels over the whole target, two triangles per cell (fb_bench.grid_mesh), all of draw 0 at depth 0.5
    grid = []
    if os.environ.get('BR_GRID') and not layers:
        side = math.sqrt(2.0 * float(os.environ['BR_GRID']))
        gc, gr = max(1, round(WID / side)), max(1, round(HGT / side))
        gv = lambda i, j: (F(int(WID * 16 * i / gc), 16), F(int(HGT * 16 * j / gr), 16))
        for j in range(gr):
            for i in range(gc):
                a, b, dd, e = gv(i, j), gv(i + 1, j), gv(i, j + 1), gv(i + 1, j + 1)
                grid += [[a, b, e], [a, e, dd]]
        ntri = len(grid)
    tris = []
    for t in range(2 * layers if layers else ntri):
        if layers:
            d = draws[0]; k = t // 2
            if os.environ.get('BR_ORDER', 'front') == 'back': k = layers - 1 - k
            zk = 0.1 + 0.8 * k / layers
            P = [(F(0), F(0)), (F(WID), F(0)), (F(0), F(HGT))] if t % 2 == 0 else [(F(WID), F(0)), (F(WID), F(HGT)), (F(0), F(HGT))]
            a2 = (P[1][0] - P[0][0]) * (P[2][1] - P[0][1]) - (P[2][0] - P[0][0]) * (P[1][1] - P[0][1])
            zs = [enc(F(zk + 0.01 * (P[i][0] / WID) + 0.02 * (P[i][1] / HGT)).limit_denominator(1 << 20)) for i in range(3)]
        elif grid:
            d = draws[0]; P = grid[t]
            a2 = (P[1][0] - P[0][0]) * (P[2][1] - P[0][1]) - (P[2][0] - P[0][0]) * (P[1][1] - P[0][1])
            zs = [enc(F(1, 2))] * 3
        else:
          # BR_DRAW0=1 (the OpenArena-like sparse-quad frame): every triangle in draw 0
          d = draws[0] if (t < 3 or os.environ.get('BR_DRAW0') == '1') else rng.choice(draws)
          # BR_SMALL=1: every triangle small (2–14 px); the draw is still made, so the
          # random state and every other frame stay as they were
          big = rng.randrange(4) == 0 and os.environ.get('BR_SMALL') != '1'
          while True:
            cx, cy = rng.uniform(0, WID), rng.uniform(0, HGT)
            r = rng.uniform(10, 50) if big else rng.uniform(2, 14)
            P = [(F(int((cx + rng.uniform(-r, r)) * 16)), F(int((cy + rng.uniform(-r, r)) * 16))) for _ in range(3)]
            P = [(x / 16, y / 16) for x, y in P]
            a2 = (P[1][0] - P[0][0]) * (P[2][1] - P[0][1]) - (P[2][0] - P[0][0]) * (P[1][1] - P[0][1])
            if a2 != 0: break
          zs = [enc(F(rng.uniform(0, 1))) for _ in range(3)]
          if rng.randrange(4) == 0: zs = [zs[0]] * 3
        qv = [rcp(enc(F(rng.uniform(0.3, 30)))) for _ in range(3)]
        lo = -1.2 if d.prog == 'b_fs_cube' else 0                 # cube directions of any sign
        fv = [[[enc(F(rng.uniform(lo, 1.2))) for _ in range(4)] for _ in range(3)] for _ in range(8)]
        if d.prog == 'b_fs_tex2p':                                    # row 0's zw = row 1's xy: b_fs_tex2m's lightmap coordinate
            for v in range(3): fv[0][v][2:4] = fv[1][v][0:2]
        # BR_UVSCREEN=k: the texture coordinates follow the WINDOW position, k texture repeats across the
        # target (rows 0 and 1 xy, row 0 zw) — texel density k x (texture size / target size), 1 texel per pixel for a 256²
        # texture on a 256-wide target, as a game's world surfaces; neighbouring triangles then share texels, random
        # coordinates do not. The random values above are still drawn, so frames without it are unchanged.
        uvs = os.environ.get('BR_UVSCREEN')
        if uvs:
            kk = F(uvs)
            for v in range(3):
                u, w = enc(P[v][0] * kk / WID), enc(P[v][1] * kk / HGT)
                fv[0][v][0:4] = [u, w, u, w]
                fv[1][v][0:2] = [u, w]
        dzdx, dzdy = dz_grad(zs, P, a2)                               # as A's setup computes them
        tc = tri_consts(zs, qv, dzdx, dzdy, P)
        T = Draw(); T.id = 1000 + t; T.d = d; T.P = P; T.a2 = a2; T.tc = tc; T.front = 1 if a2 > 0 else 0
        T.er = ref_edges(P, tc['s'])                                  # the TRI item's reference edge values
        T.var = {i: (fv[i][0], [sub(fv[i][1][c], fv[i][0][c]) for c in range(4)],
                     [sub(fv[i][2][c], fv[i][0][c]) for c in range(4)]) for i in range(8) if (d.mask >> i) & 1}
        T.o24 = poly_offset24(d.st['pofac'], d.st['pounits'], dzdx, dzdy, dbits) if d.st['po'] else 0
        tris.append(T)
    def tri_words(T):
        import zq_ref
        w = [(2 << 28) | (T.front << 25) | (T.d.mask << 8), T.id, T.d.id, T.tc['R']] + [T.tc[k] for k in TRI_KEYS] \
            + [v & 0xFFFFFFFF for v in T.tc['AB']] + zq_ref.tri_words(T.tc) \
            + T.er + [0]                                              # w18..w21: the depth bounds (zq_tri); w22..w25: the references
        for i in range(8):
            if i in T.var:
                f0, D1, D2 = T.var[i]
                v = sum(f0[c] << (24 * c) for c in range(4)) | sum(D1[c] << (96 + 24 * c) for c in range(4)) | \
                    sum(D2[c] << (192 + 24 * c) for c in range(4))
                w += [(v >> (32 * k)) & 0xFFFFFFFF for k in range(10)]
        return w
    def bary(T, px, py):
        (x0, y0), (x1, y1), (x2, y2) = T.P
        cx, cy = F(2 * px + 1, 2), F(2 * py + 1, 2)
        m1 = ((cx - x0) * (y2 - y0) - (x2 - x0) * (cy - y0)) / T.a2
        m2 = ((x1 - x0) * (cy - y0) - (cx - x0) * (y1 - y0)) / T.a2
        return 1 - m1 - m2, m1, m2
    # ---- the pass: clear mode, initial target (stored pixels of the format), initial depth-stencil image ----
    bpl = WID * bpp // 32                                     # bursts per line (the pitch is tight)
    fbp = [[rng.getrandbits(8 * bpp) for _ in range(WID)] for _ in range(HGT)]   # stored pixels
    fb = [[unpack_px(fbp[y][x], fmt) for x in range(WID)] for y in range(HGT)]   # as cell colour words
    for y in range(HGT):
        for k in range(bpl):
            ddr[B_RT_BASE + y * bpl * 8 + 8 * k] = sum(fbp[y][k * 8 * ppw + j] << (8 * bpp * j) for j in range(8 * ppw))
    dsi = None
    if ds:                                                    # {stencil, depth} words, depth within the attachment's bits
        dsi = [[rng.randrange(256) << 24 | rng.choice([umax(dbits), rng.randrange(1 << dbits)]) for _ in range(WID)]
               for _ in range(HGT)]
        for y in range(HGT):
            for k in range(WID // 8):
                ddr[B_DSI_BASE + y * WID + 8 * k] = sum(dsi[y][8 * k + j] << (32 * j) for j in range(8))
    cl = dict(c16=sum(rng.randrange(one_of(nb[c]) + 1) << (13 * (3 - c)) for c in range(4)), dither=rng.randrange(2),
              en=rng.choice([1, 1, 0]), sc=rng.choice([0, 0, 1]), cmask=rng.choice([15, 15, rng.randrange(16)]),
              x0=rng.randrange(20), y0=rng.randrange(10), depth=rng.choice([umax(dbits), rng.randrange(1 << dbits)]),
              stencil=rng.randrange(4), den=rng.choice([1, 1, 0]), sen=rng.choice([1, 1, 0]),
              smask=rng.choice([0xFF, 0xFF, rng.randrange(256)]))
    cl['x1'] = cl['x0'] + rng.randrange(10, 40); cl['y1'] = cl['y0'] + rng.randrange(4, 20)
    if layers: cl.update(depth=umax(dbits), den=1, sc=0)        # the layers start from a cleared, farthest depth
    if os.environ.get('BR_NOCLEAR') == '1': cl.update(en=0, den=0, sen=0)
    if os.environ.get('BR_CLEARSC') == '1': cl.update(en=1, sc=1, cmask=15)    # a colour clear in its scissor box
    def dclear(x, y):
        t = bayer(x, y) if cl['dither'] else 0
        return sum((((cl['c16'] >> ((3 - c) * 13)) & 0x1FFF) + t >> 4) << ((3 - c) * 8) for c in range(4))
    # ---- stream + reference render ----
    beats = []
    def emit(words):
        words = words + [0] * (len(words) & 1)
        for k in range(0, len(words), 2): beats.append(words[k] | words[k + 1] << 32)
    sent = set(); nq = 0
    flog = []                                   # fragment log: Q qx qy tri draw / F x y c16 z (tb_b_render FRAGLOG)
    # BR_SKIPBANDS=1: the bands board A walks with FLUSH flags bit 10 — those with a quad, or that the clear reaches (it writes
    # back colour under a non-zero mask, or depth / stencil under a non-zero mask into the depth-stencil image, in one of the
    # band's rows); every BAND item names the next band walked
    skipb = os.environ.get('BR_SKIPBANDS') == '1'
    cl_any = bool(cl['en'] and cl['cmask']) or bool(ds and (cl['den'] or (cl['sen'] and cl['smask'])))
    def band_has_quad(y0):
        for T in tris:
            bx0, bx1 = math.floor(min(p[0] for p in T.P)), math.ceil(max(p[0] for p in T.P))
            by0, by1 = math.floor(min(p[1] for p in T.P)), math.ceil(max(p[1] for p in T.P))
            for qy in range(max(y0 // 2, by0 // 2 - 1), min((y0 + NL) // 2, by1 // 2 + 2)):
                for qx in range(max(0, bx0 // 2 - 1), min(WID // 2, bx1 // 2 + 2)):
                    if quad_cover(T.P, qx, qy, WID, HGT): return True
        return False
    walk = [y0 for y0 in range(0, HGT, NL)
            if not skipb or band_has_quad(y0) or (cl_any and (not cl['sc'] or (y0 < cl['y1'] and y0 + NL > cl['y0'])))]
    nskip = len(range(0, HGT, NL)) - len(walk)
    for wi, y0 in enumerate(walk):
        if skipb:
            ny = walk[wi + 1] if wi + 1 < len(walk) else None
            emit([(4 << 28) | (1 << 27) | ((ny is not None) << 26) | ((ny if ny is not None else 0) << 15) | y0, 0])
        else:
            emit([(4 << 28) | y0, 0])
        cm32 = sum((0xFF << ((3 - c) * 8)) for c in range(4) if (cl['cmask'] >> (3 - c)) & 1)
        bc = [[0] * WID for _ in range(NL)]; bz = [[0] * WID for _ in range(NL)]; bs = [[0] * WID for _ in range(NL)]
        for l in range(NL):
            for x in range(WID):
                y = y0 + l
                inbox = not cl['sc'] or (cl['x0'] <= x < cl['x1'] and cl['y0'] <= y < cl['y1'])
                bc[l][x] = ((dclear(x, y) & cm32) | (fb[y][x] & ~cm32 & 0xFFFFFFFF)) if (cl['en'] and inbox) else fb[y][x]
                if ds:
                    bz[l][x] = cl['depth'] if (cl['den'] and inbox) else dsi[y][x] & 0xFFFFFF
                    s0 = dsi[y][x] >> 24
                    bs[l][x] = ((cl['stencil'] & cl['smask']) | (s0 & ~cl['smask'] & 0xFF)) if (cl['sen'] and inbox) else s0
                else:
                    bz[l][x] = cl['depth']; bs[l][x] = cl['stencil']
        band = (bc, bz, bs)
        cur = None
        for T in tris:
            quads = []
            # only the quads of the triangle's bounding box, one quad of margin, are tested — a covered pixel's
            # centre lies inside the box — so the same quads come out in the same order, without testing the whole band
            bx0, bx1 = math.floor(min(p[0] for p in T.P)), math.ceil(max(p[0] for p in T.P))
            by0, by1 = math.floor(min(p[1] for p in T.P)), math.ceil(max(p[1] for p in T.P))
            for qy in range(max(y0 // 2, by0 // 2 - 1), min((y0 + NL) // 2, by1 // 2 + 2)):
                for qx in range(max(0, bx0 // 2 - 1), min(WID // 2, bx1 // 2 + 2)):
                    cov = quad_cover(T.P, qx, qy, WID, HGT)       # pixel centres, the tie rule (as A's rasterizer)
                    if cov: quads.append((qx, qy, cov, None))
            if not quads: continue
            # the TRI item whenever the triangle is not the current one — in a later band too (board A sends the TRI
            # again and B keeps no record store)
            if cur is not T: emit(tri_words(T)); sent.add(T.id)
            cur = T
            st = T.d.st
            for qx, qy, cov, mus in quads:
                eq = quad_edges_ref(T.er, T.tc['AB'], qx, qy)             # the quad's pixel-0 edge values, as B forms them
                emit([(1 << 28) | (cov << 24) | (qy << 12) | qx])          # the QUAD item: one beat
                nq += 1
                # reference: quad setup, varyings, shader, ROP
                varys = [[0] * 16 for _ in range(16)]
                qsr = []; mue = []
                for p in range(4):
                    px, py = 2 * qx + (p & 1), 2 * qy + (p >> 1)
                    o = quad_setup_e(T.tc, eq, p, px, py, T.o24, dbits); qsr.append(o); mue.append(o['mu'])
                    for i, (f0, D1, D2) in T.var.items():
                        v = ldv(o['l1'], o['l2'], f0, D1, D2)
                        for c in range(4): varys[i][p * 4 + c] = v[c]
                    for c, b in enumerate((o['fcx'], o['fcy'], o['fcz'], o['fw'])): varys[8][p * 4 + c] = b
                    varys[9][p * 4] = ONE if T.front else 0
                    varys[10][p * 4], varys[10][p * 4 + 1], varys[10][p * 4 + 3] = mue[p][1], mue[p][2], ONE
                params = T.d.uni + [[0] * 4] * (64 - len(T.d.uni))
                # the quad as lanes 0..3 of a partial warp of the scalar core; the rows are the pre-interpolated
                # varyings (interp=False: LDV i reads them; rows 9 / 10 carry what the core forms, 11..15 zeros)
                varin = [[varys[i][p * 4: p * 4 + 4] for i in range(16)] for p in range(4)] + [[[0] * 4] * 16 for _ in range(12)]
                res = isa64.simulate_fs2(T.d.words, isa64.FSInputs(params, varin, active=0xF, interp=False, tex_fn=tex_fn), units)
                outc = [res.outc[p][c] for p in range(4) for c in range(4)]
                killed = res.killed; outz = None if res.outz is None else res.outz[:4]
                flog.append(f"Q {qx:x} {qy:x} {T.id:x} {T.d.id:x} " +
                            "".join(f"{qsr[p]['l2']:06x}{qsr[p]['l1']:06x}" for p in (3, 2, 1, 0)))
                for p in range(4):
                    if (cov >> p) & 1 and not (killed >> p) & 1:
                        px, py = 2 * qx + (p & 1), 2 * qy + (p >> 1)
                        c16 = sum(f24_cn(outc[p * 4 + c], nb[c]) << ((3 - c) * 13) for c in range(4))
                        sa = (f24_cn(outc[p * 4 + 3], nb[1]), f24_cn(outc[p * 4 + 3], nb[0]))
                        z = f24_u24(outz[p], dbits) if outz is not None else qsr[p]['d24']
                        dpass = (not st['depth_test']) or [False, z < band[1][py - y0][px], z == band[1][py - y0][px],
                                                           z <= band[1][py - y0][px], z > band[1][py - y0][px],
                                                           z != band[1][py - y0][px], z >= band[1][py - y0][px],
                                                           True][st['depth_func']]
                        flog.append(f"F {px:x} {py:x} {c16:x} {z:x} {int(dpass)}")
                        rop_frag(st, band, px, py, py - y0, c16, z, T.front, fmt, sa)
        for l in range(NL):
            fb[y0 + l] = bc[l][:]
            if ds: dsi[y0 + l] = [bs[l][x] << 24 | bz[l][x] for x in range(WID)]
    emit([5 << 28, 0])
    with open(os.path.join(GEN, "br_stream.hex"), "w") as fh: fh.write("\n".join(f"{b:016x}" for b in beats) + "\n")
    with open(os.path.join(GEN, "br_model_frags.txt"), "w") as fh: fh.write("\n".join(flog) + "\n")
    with open(os.path.join(GEN, "br_ddr.hex"), "w") as fh:
        fh.write("\n".join(f"{a:08x} " + " ".join(f"{(d >> (64 * k)) & ((1 << 64) - 1):016x}" for k in (3, 2, 1, 0))
                           for a, d in sorted(ddr.items())) + "\n")       # xsim's %h reads at most 64 bits
    with open(os.path.join(GEN, "br_skip.hex"), "w") as fh:            # tb_b_render +SKIPCHK: {may skip [31], the program's word address}
        fh.write("\n".join(f"{(int(tex_skip_ok(d.words)) << 31) | d.paddr:08x}" for d in draws) + "\n")
    if os.environ.get('BR_SKIPTEST') == '1':
        print("  sample-skip verdicts: " + ", ".join(f"{d.prog[5:]} {'skip' if tex_skip_ok(d.words) else 'all'}" for d in draws))
    with open(os.path.join(GEN, "br_expect.hex"), "w") as fh:          # stored pixels (32 or 16 bits)
        fh.write("\n".join(f"{pack_px(fb[y][x], fmt):0{2 * bpp}x}" for y in range(HGT) for x in range(WID)) + "\n")
    with open(os.path.join(GEN, "br_expect_ds.hex"), "w") as fh:       # the depth-stencil image after the frame
        if ds: fh.write("\n".join(f"{dsi[y][x]:08x}" for y in range(HGT) for x in range(WID)) + "\n")
    with open(os.path.join(GEN, "br_params.hex"), "w") as fh:
        fh.write(" ".join(f"{v:x}" for v in (B_RT_BASE, bpl, WID, HGT, cl['c16'], cl['dither'], cl['en'],
                                               cl['sc'], cl['cmask'], cl['x0'], cl['y0'], cl['x1'], cl['y1'], cl['depth'],
                                               cl['stencil'], len(beats),
                                               fmt, d16, ds, B_DSI_BASE, WID * 4 // 32, cl['den'], cl['sen'], cl['smask']))
                 + "\n")
    # ---- the same frame as host commands for B (tb_board_b3d: CMD packets over the board link + the render stream) ----
    # buffers 0x8000 | i hold the programs, images: 0xC000 = the render target, 0xC001 / 0xC002 the textures; raw data
    # (texels, the target's initial image) by MEM_WRITE (board 1); every draw by DRAW_STATE; FLUSH; FENCE 0x5EED.
    cmds = []
    hseq = [0]
    def cmd(t, body):
        hseq[0] += 1
        cmds.append([(t << 16) | (4 * len(body)), hseq[0]] + body)
    tfmt = {0: 0x00, 1: 0x05, 2: 0x06, 3: 0x04}                  # TMU code -> storage format
    for i, d in enumerate(draws):
        cmd(0x10, [0x8000 | i, d.paddr * 4, 4 * 4096])            # CREATE_BUFFER (16 KB)
        w = list(d.w32)
        for k in range(0, len(w), 256): cmd(0x20, [0x8000 | i, 4 * k] + w[k:k + 256])   # WRITE_BUFFER
    raw = {}
    for a, v in ddr.items():
        if a >= B_RT_BASE: raw[a] = v                              # texels, the target's and the depth-stencil image's initial contents
    for a in sorted(raw):
        cmd(0x04, [1, a * 4] + [(raw[a] >> (32 * k)) & 0xFFFFFFFF for k in range(8)])   # MEM_WRITE one burst
    cmd(0x11, [0xC000, B_RT_BASE * 4, WID * bpp | tfmt[fmt] << 16 | 1 << 24, WID | HGT << 16, 1 | 1 << 8])   # target (GL origin)
    if ds: cmd(0x11, [0xC003, B_DSI_BASE * 4, WID * 4 | 0x10 << 16 | 1 << 24, WID | HGT << 16, 1 | 1 << 8])   # D24S8
    for j, d in enumerate(tex):
        tbpp = 4 if d['fmt'] == 0 else 2
        cmd(0x11, [0xC001 + j, d['base'] * 4, d['w'] * tbpp | tfmt[d['fmt']] << 16, d['w'] | d['h'] << 16,
                   d['levels'] | (6 if d['cube'] else 1) << 8])
    for d in draws:
        sw = pack_state(d.st, 0, 0, 0)
        samp = [(tex[j]['incomplete'] << 24 | tex[j]['wrap_t'] << 22 | tex[j]['wrap_s'] << 20 | tex[j]['filt'] << 16 |
                 (0xC001 + j)) for j in range(2)] + [0] * 6
        body = [d.id] + sw[:11] + [(len(d.words) << 16) | (0x8000 | draws.index(d)), 0, len(d.uni), sw[13]] + samp
        for u in d.uni: body += [u[c] for c in range(4)]
        cmd(0x50, body)
    cmd(0x41, [0xC000 | (cl['en'] | cl['sc'] << 1 | cl['cmask'] << 2 | cl['dither'] << 6 | cl['den'] << 7 | cl['sen'] << 8
                         | d16 << 9) << 16,
               ((cl['c16'] >> 39) & 0x1FFF) | ((cl['c16'] >> 26) & 0x1FFF) << 13,
               ((cl['c16'] >> 13) & 0x1FFF) | (cl['c16'] & 0x1FFF) << 13,
               cl['depth'] | cl['stencil'] << 24, cl['x0'] | cl['y0'] << 16, cl['x1'] | cl['y1'] << 16,
               (0xC003 if ds else 0) | cl['smask'] << 16])
    cmd(0x40, [0x5EED])
    with open(os.path.join(GEN, "bl_cmds.hex"), "w") as fh:
        for c in cmds: fh.write(f"{len(c):x} " + " ".join(f"{w:08x}" for w in c) + "\n")
        fh.write("0\n")
    print(f"frame seed {seed}: {ntri} triangles, {nq} quads, {len(beats)} beats, {len(ddr)} DDR bursts, {len(cmds)} commands,"
          f" target {['RGBA8', 'RGBA4444', 'RGBA5551', 'RGB565'][fmt]} depth {dbits} bits{' + depth-stencil image' if ds else ''}"
          + (f", {nskip} of {len(range(0, HGT, NL))} bands left out" if skipb else "") + f" -> tools/gen/br_*.hex, bl_cmds.hex")
    if os.environ.get('BR_VERBOSE'):
        for d in draws:
            print(f"  draw {d.id} {d.prog}: uniforms " + " ".join("".join(f"{c:06x}" for c in reversed(u)) for u in d.uni))
        for T in tris:
            f0 = T.var.get(0, ([0] * 4,))[0]
            print(f"  tri {T.id} draw {T.d.id} front {T.front} v0.f0 " + "".join(f"{c:06x}" for c in reversed(f0)))


def gen_aframe(seed, NL=None):
    """The frame of board A's draw <seed> as board B must render it (the two-board bench tb_two_boards3d): the render
    stream A produces (tools/gen/as_<seed>_stream.hex, a_raster_ref.py) is parsed — TRI items give the triangles'
    constants and varyings, QUADs the covered pixels with their edge values — and rendered with the same reference as
    gen_frame (quad_setup_e, LDV, the scalar FS, the b_rop rules). One draw (the A draw's id from as_<seed>_state.mem,
    program b_fs_two: varyings 0 / 1, no textures), a random ROP state, target = the draw's width x height in a format
    drawn from the seed (or BR_FMT / BR_D16 / BR_DS), random initial pixels. Writes tools/gen/ab_<seed>_expect.hex
    (stored pixels), ab_<seed>_ddr.hex (B's initial DDR: program, bucket, target, depth-stencil image),
    ab_<seed>_params.hex (as br_params), ab_<seed>_cmds.hex (B's host commands: CREATE/WRITE_BUFFER, MEM_WRITE,
    CREATE_IMAGE, DRAW_STATE, then FLUSH and FENCE — the bench inserts A's draw before the FLUSH)."""
    import isa64
    rng = random.Random(seed * 101 + 5)
    units = isa64.Units(exact_rcp=True)
    st_a = int(open(os.path.join(GEN, f"as_{seed}_state.mem")).read().split()[0], 16)
    WID, HGT, draw_id = (st_a >> 112) & 0xFFF, (st_a >> 124) & 0xFFF, (st_a >> 99) & 0x1FFF
    if NL is None: NL = band_lines(WID)
    beats = [int(ln, 16) for ln in open(os.path.join(GEN, f"as_{seed}_stream.hex")) if ln.strip()]
    cfg = random.Random(seed * 7919 + 13)
    fmt = int(os.environ.get('BR_FMT', cfg.choice([0, 0, 1, 2, 3])))
    d16 = int(os.environ.get('BR_D16', cfg.choice([0, 0, 1])))
    ds = int(os.environ.get('BR_DS', cfg.choice([0, 1, 1])))
    # board A's coarse depth / lazy TRI items (a_setup_ref.py --cz, --zc / --zcd, --lazy): the frame's depth as its FLUSH gives
    # it ("d16 ds cden csc cdepth lazy"), and each draw's flags from its records (header word 2: did [12:0], flags [15:13]) —
    # the draws' ROP states below then keep the promise the flags make
    czp = os.path.join(GEN, f"as_{seed}_cz.hex"); cz = None; zmap = {}
    if os.path.exists(czp):
        cz = [int(v, 0) for v in open(czp).read().split()]
        d16, ds = cz[0], cz[1]
        for ln in open(os.path.join(GEN, f"as_{seed}_rec.hex")):
            t = ln.split()
            if len(t) > 6: w2 = int(t[6], 16); zmap[w2 & 0x1FFF] = (w2 >> 13) & 7
    dbits = 16 if d16 else 24
    nb = CHN[fmt]; bpp = FMT_BPP[fmt]; ppw = 4 // bpp
    assert (WID * bpp) % 32 == 0, "the target's pitch must be a multiple of 32 bytes"
    ddr = {}
    # the draw: program b_fs_two, every uniform it reads, a random ROP state (as gen_frame's rstate)
    lines = open(os.path.join(os.path.dirname(__file__), '..', 'tests', 'b_fs_two.sasm')).read().split('\n')
    words, _ = isa64.assemble(lines)
    d = Draw(); d.id = draw_id; d.words = words
    d.w32 = [x for w in words for x in (w & 0xFFFFFFFF, w >> 32)]
    # the frame's draws (a_setup_ref.py --ndraw: ids draw_id, draw_id + 1, ...): each gets the same program, uniforms and
    # ROP state under its own id
    ids = []; sw_ = [w for b in beats for w in (b & 0xFFFFFFFF, b >> 32)]; i = 0
    while i < len(sw_):
        k = sw_[i] >> 28
        if k == 2:
            if sw_[i + 2] & 0x1FFF not in ids: ids.append(sw_[i + 2] & 0x1FFF)
            i += 26 + 10 * bin((sw_[i] >> 8) & 0xFF).count('1')          # 13 + 5 nv beats
        else: i += 2                                                       # QUAD, BAND, FRAME_END: one beat each
    assert ids and ids[0] == draw_id and ids == [(draw_id + k) & 0x1FFF for k in range(len(ids))], f"the stream's draws {ids}"
    nu = 1 + max([w & 0xFF for w in words if (w >> 57) == 16] + [0])
    # the user varyings the program reads (LDV i, i < 8, bit 3 clear): every TRI of the stream must carry them — a varying
    # the draw's mask omits is never written into B's triangle store (undefined on the hardware, 0 in this model), so such
    # a draw is refused here rather than modelled (a_setup_ref.py --fsmask; fbproto.vs_state's contract)
    reads = {w & 7 for w in words if (w >> 57) == 17 and not (w >> 3) & 1}
    d.uni = [[enc(F(rng.uniform(0.0, 1.5))) for _ in range(4)] for _ in range(nu + rng.randrange(0, 3))]
    st = dict(depth_test=rng.choice([0, 1, 1]), depth_mask=rng.choice([0, 1, 1]), depth_func=rng.choice([1, 1, 3, 4, 6, 7, 5, 2]),
              stencil=rng.choice([0, 0, 1]), blend=rng.choice([0, 1]), dither=rng.choice([0, 1]),
              scissor=rng.choice([0, 0, 0, 1]), po=rng.choice([0, 0, 1]),
              cmask=rng.choice([15, 15, 15, rng.randrange(16)]),
              sref=[rng.randrange(4), rng.randrange(4)], svm=[255, rng.randrange(256)], swm=[255, rng.randrange(256)],
              sfn=[rng.randrange(8), rng.randrange(8)], osf=[rng.randrange(8) for _ in range(2)],
              odf=[rng.randrange(8) for _ in range(2)], odp=[rng.randrange(8) for _ in range(2)],
              bsf=rng.randrange(15), bdf=rng.randrange(14), bsfa=rng.randrange(15), bdfa=rng.randrange(14),
              bop=rng.randrange(5), bopa=rng.randrange(5),
              sx0=rng.randrange(WID // 4), sy0=rng.randrange(HGT // 4), pofac=enc(F(rng.uniform(-2, 2))),
              pounits=rng.randrange(-200, 200))
    cf = [rng.choice([0.0, 1.0, rng.random()]) for _ in range(4)]
    cu = [round(cf[c] * one_of(nb[c])) for c in range(4)]
    st['bcc'] = sum(cu[c] << (13 * (3 - c)) for c in range(4))
    st['bca'] = round(cf[3] * one_of(nb[0])) | round(cf[3] * one_of(nb[1])) << 13
    st['sx1'] = st['sx0'] + rng.randrange(WID // 2, WID); st['sy1'] = st['sy0'] + rng.randrange(HGT // 2, HGT)
    # board A walks a draw's rectangle alone (a_setup_ref.py --sc, the top 48 bits of as_<seed>_state.mem): the
    # stream does not carry what the scissor box masks, so B's draw state must hold that box, as the driver's does (the
    # one box for the frame's draws: --scalt frames are for board A's benches only)
    asc = tuple((st_a >> (296 + 12 * i)) & 0xFFF for i in range(4))
    a_rect = asc != (0, 0, 0, 0) and asc != (0, 0, WID, HGT)
    if a_rect:
        st.update(scissor=1, sx0=asc[0], sy0=asc[1], sx1=asc[2], sy1=asc[3])
    d.st = st
    d.paddr = B_PROG_BASE
    for k, b in enumerate(words_to_bursts(list(d.w32))): ddr[d.paddr + 8 * k] = b
    # each draw's state: the random one, or with coarse depth the one its flags promise (zmap: bit 0 CULL, bits 1:0 = 3 OCC,
    # = 2 INV, bit 2 STRICT). CULL: depth test LESS (LEQUAL or EQUAL when STRICT), no stencil, no polygon offset; OCC also depth
    # writes and no scissor box but A's rectangle (the program, b_fs_two, has no DISCARD / STOZ); INV: writes with a function
    # that may raise the depth. A draw with no flag must not raise the depth: its writes go off unless its function only lowers.
    prog_kill = any(((w >> 57) & 0x7F) in (isa64.OPS['DISCARD'], isa64.OPS['STOZ']) for w in words)
    states = {}
    for di in ids:
        s2 = dict(st)
        if cz is not None:
            z = zmap.get(di, 0)
            zr = random.Random(seed * 131 + di)
            if z & 1:
                s2.update(depth_test=1, depth_func=(zr.choice([3, 2]) if z & 4 else 1), stencil=0, po=0)
                if z & 2:
                    assert not prog_kill, "an OCC draw's program must not discard or write its depth"
                    s2['depth_mask'] = 1
                    if not a_rect: s2['scissor'] = 0
            elif z == 2:
                s2.update(depth_test=1, depth_mask=1, depth_func=zr.choice([4, 5, 6, 7]))
            elif s2['depth_mask'] and (not s2['depth_test'] or s2['depth_func'] >= 4):
                s2['depth_mask'] = 0
        states[di] = s2
        sw = pack_state(s2, d.paddr, len(words), len(d.uni))
        bucket = words_to_bursts(sw) + [0, 0, 0, 0]
        for k in range(0, len(d.uni), 2):
            u = d.uni[k:k + 2] + [[0] * 4] * (2 - len(d.uni[k:k + 2]))
            bucket.append(sum(u[h][c] << (h * 128 + 32 * c) for h in range(2) for c in range(4)))
        for k, b in enumerate(bucket): ddr[B_DS_BASE + di * 512 + 8 * k] = b
        if di == draw_id: sw0 = sw
    sws = {di: pack_state(states[di], d.paddr, len(words), len(d.uni)) for di in ids}
    # the target, the depth-stencil image, the clear (as gen_frame)
    bpl = WID * bpp // 32
    fbp = [[rng.getrandbits(8 * bpp) for _ in range(WID)] for _ in range(HGT)]
    fb = [[unpack_px(fbp[y][x], fmt) for x in range(WID)] for y in range(HGT)]
    for y in range(HGT):
        for k in range(bpl):
            ddr[B_RT_BASE + y * bpl * 8 + 8 * k] = sum(fbp[y][k * 8 * ppw + j] << (8 * bpp * j) for j in range(8 * ppw))
    dsi = None
    if ds:
        dsi = [[rng.randrange(256) << 24 | rng.choice([umax(dbits), rng.randrange(1 << dbits)]) for _ in range(WID)]
               for _ in range(HGT)]
        for y in range(HGT):
            for k in range(WID // 8):
                ddr[B_DSI_BASE + y * WID + 8 * k] = sum(dsi[y][8 * k + j] << (32 * j) for j in range(8))
    cl = dict(c16=sum(rng.randrange(one_of(nb[c]) + 1) << (13 * (3 - c)) for c in range(4)), dither=rng.randrange(2),
              en=rng.choice([1, 1, 0]), sc=rng.choice([0, 0, 1]), cmask=rng.choice([15, 15, rng.randrange(16)]),
              x0=rng.randrange(WID // 4), y0=rng.randrange(HGT // 4), depth=rng.choice([umax(dbits), rng.randrange(1 << dbits)]),
              stencil=rng.randrange(4), den=rng.choice([1, 1, 0]), sen=rng.choice([1, 1, 0]),
              smask=rng.choice([0xFF, 0xFF, rng.randrange(256)]))
    cl['x1'] = cl['x0'] + rng.randrange(WID // 2, WID); cl['y1'] = cl['y0'] + rng.randrange(HGT // 2, HGT)
    if cz is not None: cl.update(den=cz[2], sc=cz[3], depth=cz[4])
    def dclear(x, y):
        t = bayer(x, y) if cl['dither'] else 0
        return sum((((cl['c16'] >> ((3 - c) * 13)) & 0x1FFF) + t >> 4) << ((3 - c) * 8) for c in range(4))
    # ---- the stream, rendered: TRI -> triangle, TRISEL -> current, BAND -> band, QUAD -> the reference ----
    def s32(v): return v - (1 << 32) if v & 0x80000000 else v
    tris = {}
    cur = None; band = None; y0 = 0; nq = 0; flog = []
    def band_begin(y0):
        bc = [[0] * WID for _ in range(NL)]; bz = [[0] * WID for _ in range(NL)]; bs = [[0] * WID for _ in range(NL)]
        cm32 = sum((0xFF << ((3 - c) * 8)) for c in range(4) if (cl['cmask'] >> (3 - c)) & 1)
        for l in range(NL):
            for x in range(WID):
                y = y0 + l
                inbox = not cl['sc'] or (cl['x0'] <= x < cl['x1'] and cl['y0'] <= y < cl['y1'])
                bc[l][x] = ((dclear(x, y) & cm32) | (fb[y][x] & ~cm32 & 0xFFFFFFFF)) if (cl['en'] and inbox) else fb[y][x]
                if ds:
                    bz[l][x] = cl['depth'] if (cl['den'] and inbox) else dsi[y][x] & 0xFFFFFF
                    s0 = dsi[y][x] >> 24
                    bs[l][x] = ((cl['stencil'] & cl['smask']) | (s0 & ~cl['smask'] & 0xFF)) if (cl['sen'] and inbox) else s0
                else:
                    bz[l][x] = cl['depth']; bs[l][x] = cl['stencil']
        return (bc, bz, bs)
    def band_end(y0, band):
        bc, bz, bs = band
        for l in range(NL):
            fb[y0 + l] = bc[l][:]
            if ds: dsi[y0 + l] = [bs[l][x] << 24 | bz[l][x] for x in range(WID)]
    def run(beats):
        """render one stream into fb / dsi; returns its quads as (key, hidden): hidden = every covered pixel fails the depth
        test of the quad's draw against the depth stored when the quad arrives (what board A's coarse depth may cull)"""
        nonlocal cur, band, y0, nq
        cur = None; band = None; y0 = 0
        quads = []
        words = []
        for b in beats: words += [b & 0xFFFFFFFF, b >> 32]
        i = 0
        while i < len(words):
            w0 = words[i]; kind = w0 >> 28
            if kind == 4:
                if band is not None: band_end(y0, band)
                y0 = w0 & 0x1FFF; band = band_begin(y0); i += 2
            elif kind == 5:
                if band is not None: band_end(y0, band)
                band = None; i += 2
            elif kind == 2:
                mask = (w0 >> 8) & 0xFF; nv = bin(mask).count('1')
                T = Draw(); T.id = words[i + 1]; T.front = (w0 >> 25) & 1; T.draw = words[i + 2] & 0x1FFF
                assert T.draw in ids, f"triangle of draw {T.draw}, the frame's draws are {ids}"
                missing = {v for v in reads if not (mask >> v) & 1}
                if missing:
                    raise SystemExit(f"aframe seed {seed}: the fragment program reads varyings {sorted(reads)} but the draw's varying "
                                     f"mask {mask:02x} omits {sorted(missing)} (undefined on the hardware): regenerate the draw with "
                                     f"a_setup_ref.py --fsmask covering them")
                sc = words[i + 4:i + 12]
                T.tc = dict(z0=sc[0], dz1=sc[1], dz2=sc[2], q0=sc[3], q1=sc[4], q2=sc[5], dzdx=sc[6], dzdy=sc[7],
                            R=words[i + 3] & 0xFFFFFF, AB=[s32(v) for v in words[i + 12:i + 18]])
                T.var = {}
                T.zq = words[i + 18:i + 22]                                    # w18..w21: the depth bounds (zq_tri)
                T.er = words[i + 22:i + 25]                                    # w22..w24: the reference edge values
                j = i + 26
                for vi in range(8):
                    if (mask >> vi) & 1:
                        v = sum(words[j + k] << (32 * k) for k in range(10))
                        T.var[vi] = ([(v >> (24 * c)) & 0xFFFFFF for c in range(4)],
                                     [(v >> (96 + 24 * c)) & 0xFFFFFF for c in range(4)],
                                     [(v >> (192 + 24 * c)) & 0xFFFFFF for c in range(4)])
                        j += 10
                sT = states[T.draw]
                T.o24 = poly_offset24(sT['pofac'], sT['pounits'], T.tc['dzdx'], T.tc['dzdy'], dbits) if sT['po'] else 0
                tris[T.id] = T; cur = T; i = j
            elif kind == 3:
                cur = tris[words[i + 1]]; i += 2
            elif kind == 1:
                cov = (w0 >> 24) & 0xF; qy = (w0 >> 12) & 0xFFF; qx = w0 & 0xFFF
                T = cur; nq += 1
                eq = quad_edges_ref(T.er, T.tc['AB'], qx, qy); i += 2       # one beat; the edge values as B forms them
                varys = [[0] * 16 for _ in range(16)]
                qsr = []; mue = []
                for p in range(4):
                    px, py = 2 * qx + (p & 1), 2 * qy + (p >> 1)
                    o = quad_setup_e(T.tc, eq, p, px, py, T.o24, dbits); qsr.append(o); mue.append(o['mu'])
                    for vi, (f0, D1, D2) in T.var.items():
                        v = ldv(o['l1'], o['l2'], f0, D1, D2)
                        for c in range(4): varys[vi][p * 4 + c] = v[c]
                    for c, bb in enumerate((o['fcx'], o['fcy'], o['fcz'], o['fw'])): varys[8][p * 4 + c] = bb
                    varys[9][p * 4] = ONE if T.front else 0
                    varys[10][p * 4], varys[10][p * 4 + 1], varys[10][p * 4 + 3] = mue[p][1], mue[p][2], ONE
                params = d.uni + [[0] * 4] * (64 - len(d.uni))
                varin = [[varys[k][p * 4: p * 4 + 4] for k in range(16)] for p in range(4)] + [[[0] * 4] * 16 for _ in range(12)]
                res = isa64.simulate_fs2(d.words, isa64.FSInputs(params, varin, active=0xF, interp=False), units)
                outc = [res.outc[p][c] for p in range(4) for c in range(4)]
                killed = res.killed; outz = None if res.outz is None else res.outz[:4]
                flog.append(f"Q {qx:x} {qy:x} {T.id:x} {d.id:x} " + "".join(f"{qsr[p]['l2']:06x}{qsr[p]['l1']:06x}" for p in (3, 2, 1, 0)))
                sT = states[T.draw]
                hidden = bool(sT['depth_test'])
                for p in range(4):
                    if (cov >> p) & 1:
                        px, py = 2 * qx + (p & 1), 2 * qy + (p >> 1)
                        z = f24_u24(outz[p], dbits) if outz is not None else qsr[p]['d24']
                        sd = band[1][py - y0][px]
                        if [False, z < sd, z == sd, z <= sd, z > sd, z != sd, z >= sd, True][sT['depth_func']]: hidden = False
                quads.append(((qx, qy, T.id), hidden))
                for p in range(4):
                    if (cov >> p) & 1 and not (killed >> p) & 1:
                        px, py = 2 * qx + (p & 1), 2 * qy + (p >> 1)
                        c16 = sum(f24_cn(outc[p * 4 + c], nb[c]) << ((3 - c) * 13) for c in range(4))
                        sa = (f24_cn(outc[p * 4 + 3], nb[1]), f24_cn(outc[p * 4 + 3], nb[0]))
                        z = f24_u24(outz[p], dbits) if outz is not None else qsr[p]['d24']
                        flog.append(f"F {px:x} {py:x} {c16:x} {z:x}")
                        rop_frag(sT, band, px, py, py - y0, c16, z, T.front, fmt, sa)
            else:
                raise ValueError(f"stream item kind {kind} at word {i}")
        return quads
    import copy
    fb0, dsi0 = copy.deepcopy(fb), copy.deepcopy(dsi)
    flog0 = len(flog)
    quads = run(beats)
    # board A's coarse depth / lazy TRI items (tools/a_raster_ref.py: as_<seed>_stream_cz.hex): the culled stream must render
    # the very same target and depth-stencil image, and every quad it leaves out must fail the depth test at every pixel
    czs = os.path.join(GEN, f"as_{seed}_stream_cz.hex")
    if cz is not None and os.path.exists(czs):
        fbp_, dsip_ = copy.deepcopy(fb), copy.deepcopy(dsi)
        fb[:] = copy.deepcopy(fb0)
        if ds: dsi[:] = copy.deepcopy(dsi0)
        nq_plain = nq; del flog[flog0:]           # (the log: the stream board B gets, the culled one)
        beats_cz = [int(ln, 16) for ln in open(czs) if ln.strip()]
        kept = run(beats_cz)
        nq = nq_plain
        ks = [k for k, _ in kept]; ki = 0; culled = bad = 0
        for k, hid in quads:
            if ki < len(ks) and ks[ki] == k: ki += 1; continue
            culled += 1
            if not hid: bad += 1
        same = (fb == fbp_) and (not ds or dsi == dsip_)
        print(f"aframe seed {seed}: coarse depth culled {culled} of {len(quads)} quads ({bad} of them visible), stream "
              f"{len(beats)} -> {len(beats_cz)} beats, {'the same' if same else 'a DIFFERENT'} image"
              + ("" if ki == len(ks) else " (the culled stream is not a subsequence)"))
        if bad or not same or ki != len(ks): raise SystemExit("aframe: the coarse depth culled a visible quad")
        beats = beats_cz                          # the two-board bench runs board A with the frame's FLUSH: the culled stream
    with open(os.path.join(GEN, f"ab_{seed}_model_frags.txt"), "w") as fh: fh.write("\n".join(flog) + "\n")
    with open(os.path.join(GEN, f"ab_{seed}_ddr.hex"), "w") as fh:
        fh.write("\n".join(f"{a:08x} " + " ".join(f"{(v >> (64 * k)) & ((1 << 64) - 1):016x}" for k in (3, 2, 1, 0))
                           for a, v in sorted(ddr.items())) + "\n")
    with open(os.path.join(GEN, f"ab_{seed}_expect.hex"), "w") as fh:
        fh.write("\n".join(f"{pack_px(fb[y][x], fmt):0{2 * bpp}x}" for y in range(HGT) for x in range(WID)) + "\n")
    with open(os.path.join(GEN, f"ab_{seed}_expect_ds.hex"), "w") as fh:
        if ds: fh.write("\n".join(f"{dsi[y][x]:08x}" for y in range(HGT) for x in range(WID)) + "\n")
    with open(os.path.join(GEN, f"ab_{seed}_params.hex"), "w") as fh:
        fh.write(" ".join(f"{v:x}" for v in (B_RT_BASE, bpl, WID, HGT, cl['c16'], cl['dither'], cl['en'],
                                               cl['sc'], cl['cmask'], cl['x0'], cl['y0'], cl['x1'], cl['y1'], cl['depth'],
                                               cl['stencil'], len(beats),
                                               fmt, d16, ds, B_DSI_BASE, WID * 4 // 32, cl['den'], cl['sen'], cl['smask']))
                 + "\n")
    # B's host commands (as gen_frame's bl_cmds): resources, DRAW_STATE, then FLUSH and FENCE (A's draw goes before the FLUSH)
    cmds = []; hseq = [0]
    def cmd(t, body):
        hseq[0] += 1
        cmds.append([(t << 16) | (4 * len(body)), hseq[0]] + body)
    tfmt = {0: 0x00, 1: 0x05, 2: 0x06, 3: 0x04}
    cmd(0x10, [0x8000, d.paddr * 4, 4 * 4096])
    w = list(d.w32)
    for k in range(0, len(w), 256): cmd(0x20, [0x8000, 4 * k] + w[k:k + 256])
    # the target's and the depth-stencil image's initial contents: MEM_WRITEs of up to 32 contiguous bursts (1 KB)
    run = []
    def flush_run():
        if run: cmd(0x04, [1, run[0] * 4] + [(ddr[a] >> (32 * k)) & 0xFFFFFFFF for a in run for k in range(8)])
        run.clear()
    for a in sorted(ddr):
        if a < B_RT_BASE: continue
        if run and (a != run[-1] + 8 or len(run) == 32): flush_run()
        run.append(a)
    flush_run()
    cmd(0x11, [0xC000, B_RT_BASE * 4, WID * bpp | tfmt[fmt] << 16 | 1 << 24, WID | HGT << 16, 1 | 1 << 8])
    if ds: cmd(0x11, [0xC003, B_DSI_BASE * 4, WID * 4 | 0x10 << 16 | 1 << 24, WID | HGT << 16, 1 | 1 << 8])
    for di in ids:
        sw = sws[di]
        body = [di] + sw[:11] + [(len(d.words) << 16) | 0x8000, 0, len(d.uni), sw[13]] + [0] * 8
        for u in d.uni: body += [u[c] for c in range(4)]
        cmd(0x50, body)
    cmd(0x41, [0xC000 | (cl['en'] | cl['sc'] << 1 | cl['cmask'] << 2 | cl['dither'] << 6 | cl['den'] << 7 | cl['sen'] << 8
                         | d16 << 9) << 16,
               ((cl['c16'] >> 39) & 0x1FFF) | ((cl['c16'] >> 26) & 0x1FFF) << 13,
               ((cl['c16'] >> 13) & 0x1FFF) | (cl['c16'] & 0x1FFF) << 13,
               cl['depth'] | cl['stencil'] << 24, cl['x0'] | cl['y0'] << 16, cl['x1'] | cl['y1'] << 16,
               (0xC003 if ds else 0) | cl['smask'] << 16])
    cmd(0x40, [0x5EED])
    with open(os.path.join(GEN, f"ab_{seed}_cmds.hex"), "w") as fh:
        for c in cmds: fh.write(f"{len(c):x} " + " ".join(f"{w:08x}" for w in c) + "\n")
        fh.write("0\n")
    # tb_two_boards3d +SKIP sends this FLUSH with flags bit 10: the bands board A leaves out per frame (a_raster_ref's walk;
    # the image is the same — a band left out holds nothing to draw and nothing of the clear that is written back)
    import a_raster_ref as ar
    fw = list(cmds[-2][2:]); fw[0] |= 1 << 26
    _, _, csk = ar.stream(ar.load_records(os.path.join(GEN, f"as_{seed}_rec.hex")), WID, HGT, skip=ar.flush_skip(fw))
    with open(os.path.join(GEN, f"ab_{seed}_skip_count.hex"), "w") as fh: fh.write(f"{csk['bskip']}\n")
    print(f"aframe seed {seed}: draw{'s' if len(ids) > 1 else ''} {', '.join(map(str, ids))}, {len(tris)} triangles, {nq} quads, {len(beats)} beats, {len(ddr)} DDR bursts,"
          f" {len(cmds)} commands, target {WID} x {HGT} {['RGBA8', 'RGBA4444', 'RGBA5551', 'RGB565'][fmt]} depth {dbits} bits"
          f"{' + depth-stencil image' if ds else ''}, {csk['bskip']} bands left out with skip -> tools/gen/ab_{seed}_*")


def frag_compare(rtl_path):
    """the RTL's fragment log (tb_b_render -d FRAGLOG: L / D per launched / dropped quad in launch order, F per fragment
    given to b_rop) against the model's (Q per quad, F per fragment): the first divergence"""
    mq = []
    for ln in open(os.path.join(GEN, "br_model_frags.txt")):
        t = ln.split()
        if not t: continue
        if t[0] == 'Q': mq.append([tuple(int(v, 16) for v in t[1:5]) + (t[5],), []])
        else: mq[-1][1].append(tuple(int(v, 16) for v in t[1:]))
    ev, rf = [], []
    for ln in open(rtl_path):
        t = ln.split()
        if not t or t[0] == '#': continue
        if t[0] in ('L', 'D'): ev.append((t[0], int(t[1], 16), int(t[2], 16), t[3] if len(t) > 3 else ''))
        elif t[0] == 'F': rf.append(tuple(int(v, 16) for v in t[1:]))        # (the log's other kinds are for reading)
    fi = 0
    for qi, (q, frags) in enumerate(mq):
        if qi >= len(ev): print(f"RTL has only {len(ev)} quad events, model quad {qi} {q}"); return
        e = ev[qi]
        if (e[1], e[2]) != q[:2]: print(f"quad {qi}: RTL {e}, model {q}"); return
        if e[0] == 'D':
            if any(f[4] for f in frags):
                print(f"quad {qi} (qx {q[0]} qy {q[1]} tri {q[2]} draw {q[3]}): dropped by early depth, but a fragment passes "
                      f"the depth test: {[(f[0], f[1], hex(f[3])) for f in frags if f[4]]}")
                return
            continue
        if e[3].lstrip('0') != q[4].lstrip('0'):
            print(f"quad {qi} (qx {q[0]} qy {q[1]} tri {q[2]} draw {q[3]}): weights RTL {e[3]}, model {q[4]}"); return
        for f in frags:
            if fi >= len(rf): print(f"quad {qi} {q}: RTL fragments end"); return
            if rf[fi] != f[:4]:
                print(f"quad {qi} (qx {q[0]} qy {q[1]} tri {q[2]} draw {q[3]}) fragment {fi}: RTL x {rf[fi][0]} y {rf[fi][1]} "
                      f"c16 {rf[fi][2]:013x} z {rf[fi][3]:06x}, model x {f[0]} y {f[1]} c16 {f[2]:013x} z {f[3]:06x}")
                return
            fi += 1
    print(f"fragment logs agree: {len(mq)} quads, {fi} fragments")


def check_conv():
    """the integer forms the RTL uses: u24 -> FP24 via k * (2^24 + 1), sticky = (k != 0); FP24 -> u24 = rne(z*U24)"""
    bad = 0
    M = f24.M
    for k in range(1, 1 << 24):
        t = k * ((1 << 24) + 1)                       # k / (2^24 - 1) = (t + eps) / 2^48, 0 < eps < 1
        if k == U24:
            r = ONE
        else:
            L = t.bit_length()
            sh = L - (M + 1)
            q = t >> sh; g = (t >> (sh - 1)) & 1
            # sticky: lower bits or eps > 0 (always) -> ties impossible: round up iff the guard bit is set
            if g: q += 1
            e = L - 48 - 1
            if q == (1 << (M + 1)): q >>= 1; e += 1
            r = ((e + f24.bias) << M) | (q & ((1 << M) - 1))
        # exact RNE of k / U24 by integer division: k * 2^s / U24 with s so that the quotient has M + 1 bits
        s = M + 1 + 24 - k.bit_length()
        qq, rr = divmod(k << s, U24)
        if qq >> (M + 1): s -= 1; qq, rr = divmod(k << s, U24)
        if 2 * rr > U24 or (2 * rr == U24 and (qq & 1)): qq += 1
        ee = -s + M
        if qq >> (M + 1): qq >>= 1; ee += 1
        x = ((ee + f24.bias) << M) | (qq & ((1 << M) - 1))
        if r != x:
            bad += 1
            if bad < 5: print("u24->f24 mismatch", k, hex(r), hex(x))
        if k & 0xFFFFF == 0: print(f"  ... {k >> 20}/16", flush=True)
    print(f"u24 -> FP24: {bad} mismatches over 2^24 - 1 values")


if __name__ == "__main__":
    if len(sys.argv) >= 2 and sys.argv[1] == "qs":
        gen_qs(int(sys.argv[2]) if len(sys.argv) > 2 else 1, int(sys.argv[3]) if len(sys.argv) > 3 else 200,
               1 if len(sys.argv) > 4 and sys.argv[4] == "d16" else 0)
    elif len(sys.argv) >= 2 and sys.argv[1] == "conv":
        check_conv()
    elif len(sys.argv) >= 3 and sys.argv[1] == "fsi":
        gen_fsi(sys.argv[2])
    elif len(sys.argv) >= 2 and sys.argv[1] == "div":
        check_div()
    elif len(sys.argv) >= 3 and sys.argv[1] == "fragcmp":
        frag_compare(sys.argv[2])
    elif len(sys.argv) >= 2 and sys.argv[1] == "frame":
        gen_frame(int(sys.argv[2]) if len(sys.argv) > 2 else 1, int(sys.argv[3]) if len(sys.argv) > 3 else 20,
                  WID=int(os.environ.get('BR_WID', '64')), HGT=int(os.environ.get('BR_HGT', '32')))   # BR_WID / BR_HGT: the target
    elif len(sys.argv) >= 3 and sys.argv[1] == "aframe":
        gen_aframe(int(sys.argv[2]))
    else:
        print(__doc__)
