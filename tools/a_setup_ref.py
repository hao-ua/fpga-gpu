#!/usr/bin/env python3
"""a_setup_ref — model of board A's setup / record / binning stage (src/rtl/a_setup.sv) on a
draw of tools/a_vtx_ref.py: the primitive vertices (gl_Position, 8 varyings) go through the near-plane clip (clip_near,
w >= 2^-7, Sutherland-Hodgman with FP24 lerps), the per-vertex setup (vtx_setup2: 1/w by the SFU reciprocal, NDC,
viewport to 1/16-pixel window positions with round-half-even snapping and saturation to 16 bits, window z = ndc_z/2 +
1/2), the triangle setup (signed area in 1/256 pixel^2, facing / cull, trivial rejection outside the target, bands of
NL rows), and the TRIANGLE RECORD A keeps in its DDR (REC_BASE + id * 512 B) and streams to B: a raster header burst
then the TRI item (R, z0, dz1, dz2, q0..q2, dz/dx, dz/dy, the 6 edge steps, per read
varying f0, D1, D2). Every FP24
operation is the hardware's (float_ref / b_ref unit models); dz/dx = (dz1 * fdy2 - dz2 * fdy1) * rcp(a2 / 16) etc. with
the window deltas converted to FP24 — the formula b_ref.py adopts for its generator.

  python3 tools/a_setup_ref.py <seed> [a_vtx_ref options] [--cull 0|1|2|3] [--cw] [--fsmask HEX] [--hgt N] [--wid N] [--lw N] [--depth N F] [--z0]
     (cull 0 none, 1 back, 2 front, 3 front and back; --depth: glDepthRangef(n, f), default 0 1; a_vtx_ref's --types: client
      attribute formats — BYTE .. FLOAT, normalized, byte strides; --arrays --first N: a DrawArrays range, N may exceed 65535;
      --z0: the clip volume 0 <= z <= w (VS_STATE word 4 bit 24: a library that keeps depth reversed, far at z = 0) — plane 1
      is z >= 0 and window z = n + ndc_z (f - n), ndc_z in 0 .. 1)
     (--mode points|lines|lstrip|lloop draws points (gl_PointSize from the program) and lines of width --lw)
     -> the av_<seed> files (the draw) + tools/gen/as_<seed>_state.mem and as_<seed>_rec.hex (per emitted triangle:
        "id band_lo band_hi nwords w0 w1 ..." = the push to the binner and the record words at REC_BASE + id * 128),
        as_<seed>_cmds.hex (the draw as host commands for A's dispatcher, ingress-queue bursts; tb_a_cmd)
"""
import os, sys, random
from fractions import Fraction as F
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import float_ref as fr, isa64, b_ref, a_vtx_ref
from a_vtx_ref import build_draw, parse_args

FMT = fr.FP24; GEN = isa64.GEN
ONE = fr.encode_real(F(1), FMT); HALF = fr.encode_real(F(1, 2), FMT)
WEPS = (FMT.bias - 7) << FMT.M                                   # 2^-7, the clip plane of clip_near
def NLOF(st): return b_ref.band_lines(st['wid'])                 # rows per band (B's bands): 8 up to 1024 wide, else 4 (b_ref.band_lines)
REC_BASE = 0x1000000                                             # word address of triangle record 0 (64 MB)
REC_STRIDE = 128                                                 # words per record (512 B, as B's record store)


def mul(a, b): return fr.fmul(a, b, FMT)
def add(a, b): return fr.fadd(a, b, FMT)
def sub(a, b): return fr.fadd(a, b ^ (1 << (FMT.W - 1)), FMT)
def madd(a, b, c): return fr.madd(a, b, c, FMT)
def enc(v): return fr.encode_real(F(v), FMT)
rcp = b_ref.rcp


def ftoi_sat16(bits):
    """float_cvt_f2i (OW 18) then the 16-bit clamp of vtx_setup2: trunc toward 0, saturate; Inf / NaN -> 0"""
    r = fr.to_real(bits, FMT)
    v = int(r) if r is not None else 0
    return max(-32768, min(32767, v))


def inside(w): return (w >> (FMT.W - 1)) == 0 and (w & ((1 << (FMT.W - 1)) - 1)) >= WEPS


# ---- the clip volume (ES 2.0 §2.13): seven planes, each a signed FP24 distance d(v) >= 0 inside (clip_near.sv). Plane 0,
#      w >= 2^-7, is the guard against the division (its sign equals inside(w)); planes 1 and 2 are the near and far
#      planes -w <= z <= w; planes 3..6 are the GUARD BAND in x / y: x <= gxhi w, x >= gxlo w,
#      y <= gyhi w, y >= gylo w, where the four constants (VS_STATE words 9..12, FP24, computed by the host from the viewport)
#      are the NDC values whose window coordinate is +-2040 px, i.e. inside the 16-bit saturation of vtx_setup2 — so a
#      vertex is clipped exactly instead of being moved by the saturation. Points are discarded by the
#      first three only (§2.13: a point is passed unchanged unless outside near / far).
NPLANE = 7
NPLANE_PT = 3
GB = 32640                                                       # the guard band: +-2040 px in 1/16 pixel


def guard_consts(vp):
    """VS_STATE words 9..12 = (gxlo, gxhi, gylo, gyhi): (−GB − o) / s and (GB − o) / s per axis, ordered, as FP24"""
    out = []
    for s, o in ((vp[0], vp[1]), (vp[2], vp[3])):
        sr, orr = fr.to_real(s, FMT), fr.to_real(o, FMT)
        a, b = sorted([(-GB - orr) / sr, (GB - orr) / sr])
        out += [fr.encode_real(a, FMT), fr.encode_real(b, FMT)]
    return out


def plane_d(v, p, gb, z0=0):
    if p == 0: return sub(v[3], WEPS)        # w - 2^-7
    if p == 1: return add(v[2], 0 if z0 else v[3])   # z + w (near); z0: z + 0 — the volume 0 <= z <= w, far at z = 0
    if p == 2: return sub(v[3], v[2])        # w - z   (far)
    if p == 3: return sub(mul(gb[1], v[3]), v[0])   # gxhi w - x
    if p == 4: return sub(v[0], mul(gb[0], v[3]))   # x - gxlo w
    if p == 5: return sub(mul(gb[3], v[3]), v[1])   # gyhi w - y
    return sub(v[1], mul(gb[2], v[3]))              # y - gylo w


def dpos(d): return (d >> (FMT.W - 1)) == 0  # inside: the sign clear (the boundary is inside)


def lerp_in(vin, vout, din, dout):
    """the intersection of edge in -> out with the plane, lerped FROM THE INSIDE vertex: t = d_in / (d_in - d_out),
    new = v_in + t (v_out - v_in) — the error is bounded by ulp(|v_in|), and the two triangles sharing the edge get the
    same point whichever way they traverse it"""
    t = mul(din, rcp(sub(din, dout)))
    return [add(vin[k], mul(t, sub(vout[k], vin[k]))) for k in range(len(vin))]


def clip(tri, gb, z0=0):
    """clip_near on one triangle: vertices = [x, y, z, w, 32 varying scalars]; Sutherland-Hodgman against the seven
    planes in order (a plane every vertex is inside leaves the polygon as it is), the polygon grows to at most 6 + 2
    per further plane; returns the fan (0, i, i + 1) of the final polygon"""
    poly = list(tri)
    for p in range(NPLANE):
        d = [plane_d(v, p, gb, z0) for v in poly]
        if all(dpos(x) for x in d): continue
        out = []; n = len(poly)
        for i in range(n):
            a, b = poly[i], poly[(i + 1) % n]; da, db = d[i], d[(i + 1) % n]
            ia, ib = dpos(da), dpos(db)
            if ia: out.append(a)
            if ia != ib: out.append(lerp_in(a, b, da, db) if ia else lerp_in(b, a, db, da))
        poly = out
        if len(poly) < 3: return []
    return [(poly[0], poly[i], poly[i + 1]) for i in range(1, len(poly) - 1)]


def clip_line(va, vb, gb, z0=0):
    """a line segment against the seven planes: None when it lies outside one of them, else the (possibly moved) ends"""
    for p in range(NPLANE):
        da, db = plane_d(va, p, gb, z0), plane_d(vb, p, gb, z0)
        ia, ib = dpos(da), dpos(db)
        if not ia and not ib: return None
        if ia != ib:
            if ia: vb = lerp_in(va, vb, da, db)
            else: va = lerp_in(vb, va, db, da)
    return va, vb


def point_in(v, z0=0): return all(dpos(plane_d(v, p, None, z0)) for p in range(NPLANE_PT))


def ftoi_sat18(bits):
    """float_cvt_f2i (OW 18): trunc toward 0, saturate to 18 bits; Inf / NaN -> 0 (the unsaturated window position of a point)"""
    r = fr.to_real(bits, FMT)
    v = int(r) if r is not None else 0
    return max(-131072, min(131071, v))


def vertex_setup(v, vp):
    """vtx_setup2: (sx, sy) in 1/16 pixel (signed 16 bits), zw window z = ndc_z * zs + zo (vp[4], vp[5]: the depth range
    (f - n) / 2, (n + f) / 2), iw = 1/w, reject (w <= 0); then the 18-bit window position before the saturation (points)"""
    x, y, z, w = v[:4]
    iw = rcp(w)
    ndcx, ndcy, ndcz = mul(x, iw), mul(y, iw), mul(z, iw)
    fx = fr.fround(madd(ndcx, vp[0], vp[1]), FMT, 3); fy = fr.fround(madd(ndcy, vp[2], vp[3]), FMT, 3)
    sx, sy = ftoi_sat16(fx), ftoi_sat16(fy)
    zw = madd(ndcz, vp[4], vp[5])
    reject = (w >> (FMT.W - 1)) == 1 or ((w >> FMT.M) & ((1 << FMT.E) - 1)) == 0
    return sx, sy, zw, iw, reject, ftoi_sat18(fx), ftoi_sat18(fy)


MAXPT = 64                                                       # the largest point size (pixels); the smallest is 1
MAXLW = 64                                                       # the widest line


def clampy(yq, lo, hi): return max(lo, min(hi - 1, yq >> 4))    # the pixel row of a 1/16 coordinate, clamped to lo .. hi - 1


def rect_of(st):
    """the draw's rectangle [x0, x1) x [y0, y1) in pixels: its scissor box cut to the target (VS_STATE words
    13 / 14), the whole target without one. Board B's scissor test masks everything outside it, so a primitive with no
    pixel in it is dropped and one that crosses it is binned only into the bands the rectangle covers; an empty rectangle
    drops everything"""
    x0, y0, x1, y1 = st.get('sc', (0, 0, st['wid'], st['hgt']))
    return x0, y0, min(x1, st['wid']), min(y1, st['hgt'])


def zc_hdr(zc):
    """the draw's coarse-depth flags (VS_STATE word 4 [28:25] = {INV, STRICT, CULL, OCC}) as the record header carries
    them (word 2 [15:13], a_setup.sv zc_enc): bit 0 = cull (CULL or OCC), bit 1 = OCC with bit 0, INV without it,
    bit 2 = STRICT with bit 0. INV wins over the others (a draw that may raise the depth is never culled)."""
    occ, cull, strict, inv = zc & 1, (zc >> 1) & 1, (zc >> 2) & 1, (zc >> 3) & 1
    if inv: return 2
    c = cull | occ
    return c | (occ << 1) | ((strict & c) << 2)


def hdr_rect(hdr, st):
    """the rectangle in the raster header's spare bits, for a_render's walk: x0 = word 1 [31:20], y0 = word 2 [31:20],
    y1 = {word 0 [7:0], word 2 [19:16]}, x1 = {word 7 [31:26], word 7 [15:10]}; the coarse-depth flags in word 2 [15:13]
    (zc_hdr)"""
    x0, y0, x1, y1 = rect_of(st)
    hdr[0] |= (y1 >> 4) & 0xFF
    hdr[1] |= x0 << 20
    hdr[2] |= (y0 << 20) | ((y1 & 0xF) << 16) | (zc_hdr(st.get('zc', 0)) << 13)
    hdr[7] |= ((x1 >> 6) << 26) | ((x1 & 0x3F) << 10)
    return hdr


def zq_append(tri, line=False):
    """the TRI item's depth-bound words w18..w21 (zq_tri.sv) after its steps: from R, z0, dz1, dz2 and A1 B1 A2 B2"""
    import zq_ref
    return tri + zq_ref.zq_words(tri[3], tri[4], tri[5], tri[6], tri[14], tri[15], tri[16], tri[17], line)


def point_setup(v, ps_row, st, tid):
    """A point: dropped if its centre is outside the clip volume's w, near or far plane (ES 2.0 §2.13: a point is
    discarded, never clipped) or if its square misses the target; the size = 2 RNE(8 gl_PointSize) in 1/16 pixel — EVEN,
    so the square's centre is the point's exact window position (§3.3) — clamped to [16, 16 MAXPT]
    (undefined sizes <= 0, NaN, Inf -> the minimum); the centre is the 18-bit window position BEFORE the 16-bit saturation
    (record words 5 / 6), so a point near the +-2048 px limit of a tall target is not moved; the square
    [x - size/2, x - size/2 + size) x [y - size/2, ..); gl_PointCoord s = (cx - xl) / size, t = (yt - cy) / size"""
    if not point_in(v, st.get('z0', 0)): return None
    sx, sy, zw, iw, reject, x18, y18 = vertex_setup(v, st['vp'])
    if reject: return None
    r = fr.to_real(ps_row[0], FMT)
    ps = 16 if (r is None or r <= 0) else max(16, min(16 * MAXPT, 2 * b_ref.rne_int(r * 8)))
    half = ps >> 1
    xl, yb = x18 - half, y18 - half
    rx0, ry0, rx1, ry1 = rect_of(st)
    if rx0 >= rx1 or ry0 >= ry1: return None
    if xl + ps <= 16 * rx0 or xl >= 16 * rx1 or yb + ps <= 16 * ry0 or yb >= 16 * ry1: return None
    lo, hi = clampy(yb, ry0, ry1) // NLOF(st), clampy(yb + ps - 1, ry0, ry1) // NLOF(st)
    vm = st['fsmask']
    R = rcp(enc(ps))
    tri = [(2 << 28) | (2 << 26) | (1 << 25) | (vm << 8), tid, st['draw_id'], R, zw, 0, 0, iw, 0, 0, 0, 0] \
        + [v & 0xFFFFFFFF for v in (0, 0, 16, 0, 0, -16)]
    tri = zq_append(tri)
    for i in range(8):
        if (vm >> i) & 1:
            f0 = [v[4 + 4 * i + c] for c in range(4)]
            tri += [(sum(f0[c] << (24 * c) for c in range(4)) >> (32 * k)) & 0xFFFFFFFF for k in range(10)]
    hdr = [(0xA << 28) | (2 << 26) | (1 << 25) | (vm << 8), tid, st['draw_id'], ps,
           ((sy & 0xFFFF) << 16) | (sx & 0xFFFF), x18 & 0xFFFFFFFF, y18 & 0xFFFFFFFF, (hi << 16) | lo]
    return lo, hi, hdr_rect(hdr, st) + tri


def line_setup(va, vb, st, tid):
    """A line segment a -> b: clipped against the w, near and far planes and the x / y guard band (an outside end moved
    onto the plane, every attribute lerped from the inside end; both outside -> dropped); then the setup of both ends, the squared length
    (1/256 pixel^2, the edge shift s), the record: R = rcp(i2f(len2 >> s)), the steps of e1 = (c - a) . (b - a) (A1 =
    16 dx, B1 = 16 dy, A0 / B0 their negations, e2 = 0), z along the segment, q0 / q1 = 1/w of the ends, the varyings
    f0 = a, D1 = b - a, D2 = 0; the line width (pixels) travels in the header for the rasterizer"""
    c = clip_line(va, vb, st['gb'], st.get('z0', 0))
    if c is None: return None
    va, vb = c
    Sa, Sb = vertex_setup(va, st['vp']), vertex_setup(vb, st['vp'])
    if Sa[4] or Sb[4]: return None
    (xa, ya), (xb, yb) = (Sa[0], Sa[1]), (Sb[0], Sb[1])
    dx, dy = xb - xa, yb - ya
    len2 = dx * dx + dy * dy
    if len2 == 0: return None
    lw = max(1, min(MAXLW, st['lw']))
    M = 16 * lw
    rx0, ry0, rx1, ry1 = rect_of(st)
    if rx0 >= rx1 or ry0 >= ry1: return None
    if max(xa, xb) < 16 * rx0 - M or min(xa, xb) >= 16 * rx1 + M or max(ya, yb) < 16 * ry0 - M or min(ya, yb) >= 16 * ry1 + M: return None
    lo = clampy(min(ya, yb) - 8 * (lw - 1), ry0, ry1) // NLOF(st)
    hi = min(ry1 - 1, clampy(max(ya, yb), ry0, ry1) + lw - 1) // NLOF(st)
    s = b_ref.edge_shift(len2)
    R = rcp(enc(len2 >> s))
    A1, B1 = (16 * dx) >> s, (16 * dy) >> s
    A0, B0 = (-16 * dx) >> s, (-16 * dy) >> s
    vm = st['fsmask']
    tri = [(2 << 28) | (1 << 26) | (1 << 25) | (vm << 8), tid, st['draw_id'], R, Sa[2], sub(Sb[2], Sa[2]), 0, Sa[3], Sb[3], 0, 0, 0] \
        + [v & 0xFFFFFFFF for v in (A0, B0, A1, B1, 0, 0)]
    tri = zq_append(tri, line=True)
    for i in range(8):
        if (vm >> i) & 1:
            f0 = [va[4 + 4 * i + c] for c in range(4)]
            D1 = [sub(vb[4 + 4 * i + c], f0[c]) for c in range(4)]
            v = sum(f0[c] << (24 * c) for c in range(4)) | sum(D1[c] << (96 + 24 * c) for c in range(4))
            tri += [(v >> (32 * k)) & 0xFFFFFFFF for k in range(10)]
    hdr = [(0xA << 28) | (1 << 26) | (1 << 25) | (lw << 18) | (s << 16) | (vm << 8), tid, st['draw_id'], (len2 >> s) & 0xFFFFFFFF,
           ((ya & 0xFFFF) << 16) | (xa & 0xFFFF), ((yb & 0xFFFF) << 16) | (xb & 0xFFFF), 0, (hi << 16) | lo]
    return lo, hi, hdr_rect(hdr, st) + tri


def tri_setup(verts, st, tid):
    """one clipped triangle -> None (dropped) or (band_lo, band_hi, record words)"""
    S = [vertex_setup(v, st['vp']) for v in verts]
    if any(s[4] for s in S): return None
    (x0, y0), (x1, y1), (x2, y2) = [(s[0], s[1]) for s in S]
    a2 = (x1 - x0) * (y2 - y0) - (x2 - x0) * (y1 - y0)
    if a2 == 0: return None
    front = int(a2 > 0) ^ st['front_cw']
    if (st['cull'] == 1 and not front) or (st['cull'] == 2 and front) or st['cull'] == 3: return None   # 3 = FRONT_AND_BACK
    xs, ys = [x0, x1, x2], [y0, y1, y2]
    # the pixel columns / rows whose CENTRE (16 c + 8 in 1/16 pixel) lies in the bounding box: only
    # those can hold a covered pixel, so a
    # triangle with none, or with none inside the target, is dropped, and the bands are those of the centre rows —
    # fewer visits in the render stream, the same pixels
    cx_lo, cx_hi = (min(xs) + 7) >> 4, (max(xs) - 8) >> 4
    cy_lo, cy_hi = (min(ys) + 7) >> 4, (max(ys) - 8) >> 4
    rx0, ry0, rx1, ry1 = rect_of(st)
    if rx0 >= rx1 or ry0 >= ry1: return None
    if cx_lo > cx_hi or cy_lo > cy_hi or cx_hi < rx0 or cx_lo >= rx1 or cy_hi < ry0 or cy_lo >= ry1: return None
    lo = max(ry0, cy_lo) // NLOF(st)
    hi = min(ry1 - 1, cy_hi) // NLOF(st)
    zw = [s[2] for s in S]; q = [s[3] for s in S]
    dz1, dz2 = sub(zw[1], zw[0]), sub(zw[2], zw[0])
    fdx1, fdy1, fdx2, fdy2 = enc(x1 - x0), enc(y1 - y0), enc(x2 - x0), enc(y2 - y0)
    r = rcp(enc(F(a2, 16)))                                      # i2f(a2) with the exponent lowered by 4
    dzdx = mul(sub(mul(dz1, fdy2), mul(dz2, fdy1)), r)
    dzdy = mul(sub(mul(dz2, fdx1), mul(dz1, fdx2)), r)
    # the rasterizer's integer edge values go out >> s (32 bits), R = rcp(i2f(a2 >> s)), the per-pixel steps >> s
    s, R, AB = b_ref.tri_edges([(F(x0, 16), F(y0, 16)), (F(x1, 16), F(y1, 16)), (F(x2, 16), F(y2, 16))])
    vm = st['fsmask']
    tri = [(2 << 28) | (front << 25) | (vm << 8), tid, st['draw_id'], R, zw[0], dz1, dz2, q[0], q[1], q[2], dzdx, dzdy] \
        + [v & 0xFFFFFFFF for v in AB]
    tri = zq_append(tri)
    for i in range(8):
        if (vm >> i) & 1:
            f0 = [verts[0][4 + 4 * i + c] for c in range(4)]
            D1 = [sub(verts[1][4 + 4 * i + c], f0[c]) for c in range(4)]
            D2 = [sub(verts[2][4 + 4 * i + c], f0[c]) for c in range(4)]
            v = sum(f0[c] << (24 * c) for c in range(4)) | sum(D1[c] << (96 + 24 * c) for c in range(4)) | \
                sum(D2[c] << (192 + 24 * c) for c in range(4))
            tri += [(v >> (32 * k)) & 0xFFFFFFFF for k in range(10)]
    hdr = [(0xA << 28) | (front << 25) | (s << 16) | (vm << 8), tid, st['draw_id'], (a2 >> s) & 0xFFFFFFFF,
           ((y0 & 0xFFFF) << 16) | (x0 & 0xFFFF), ((y1 & 0xFFFF) << 16) | (x1 & 0xFFFF), ((y2 & 0xFFFF) << 16) | (x2 & 0xFFFF),
           (hi << 16) | lo]
    return lo, hi, hdr_rect(hdr, st) + tri


def check_dz_grad(n=300):
    """b_ref.dz_grad (the B model's generator) against this model's formula on random triangles"""
    rng = random.Random(3); bad = 0
    for _ in range(n):
        P, a2, z, q, zs, ws = b_ref.rand_tri(rng)
        S = [(int(p[0] * 16), int(p[1] * 16)) for p in P]; (x0, y0), (x1, y1), (x2, y2) = S
        a2q = (x1 - x0) * (y2 - y0) - (x2 - x0) * (y1 - y0)
        dz1, dz2 = sub(z[1], z[0]), sub(z[2], z[0]); r = rcp(enc(F(a2q, 16)))
        ex = mul(sub(mul(dz1, enc(y2 - y0)), mul(dz2, enc(y1 - y0))), r); ey = mul(sub(mul(dz2, enc(x1 - x0)), mul(dz1, enc(x2 - x0))), r)
        bad += b_ref.dz_grad(z, P, a2) != (ex, ey)
    print("dz_grad vs a_setup_ref:", "ok" if bad == 0 else f"{bad} of {n} differ")
    return bad == 0


def clip_vectors(nvar=8, n=40, z0=0):
    """tools/gen/clipnear_vectors.hex for sim/tb_clip_near.sv: triangles crossing the w, near and far planes (and none),
    each with its clip() result — 0 .. 4 triangles of 4 + nvar attributes; the format of float_ref.emit_clipnear"""
    rng = random.Random(777)
    hw = (FMT.W + 3) // 4
    def mkv(x, y, z, w, vb):
        return [enc(F(x).limit_denominator(1 << 14)), enc(F(y).limit_denominator(1 << 14)), enc(F(z).limit_denominator(1 << 14)),
                enc(F(w).limit_denominator(1 << 14))] + [enc(F(vb + 0.13 * k).limit_denominator(1 << 14)) for k in range(nvar)]
    tris = [[(0.2, 0.1, 0.0, 1.0, 0.5), (0.4, -0.3, 0.0, 2.0, 1.0), (0.1, 0.5, 0.0, -0.5, 1.5)],     # one vertex behind the eye (w plane)
            [(0.2, 0.1, 0.0, 1.0, 0.3), (0.4, -0.3, 0.0, -0.5, 0.7), (0.1, 0.5, 0.0, -1.0, 1.1)],    # two behind
            [(0.2, 0.1, 0.0, 1.0, 0.2), (0.4, -0.3, 0.0, 2.0, 0.6), (0.1, 0.5, 0.0, 1.5, 1.0)],      # all inside
            [(0.2, 0.1, -1.5, 1.0, 0.2), (0.4, -0.3, 0.5, 1.0, 0.6), (0.1, 0.5, 0.0, 1.0, 1.0)],     # one before the near plane
            [(0.2, 0.1, 1.5, 1.0, 0.2), (0.4, -0.3, 1.8, 1.0, 0.6), (0.1, 0.5, 0.0, 1.0, 1.0)],      # two beyond the far plane
            [(0.2, 0.1, -1.5, 1.0, 0.2), (0.4, -0.3, 1.5, 1.0, 0.6), (0.1, 0.5, 0.0, 1.0, 1.0)],     # near and far: 5 vertices
            [(0.2, 0.1, -1.5, 1.0, 0.2), (0.4, -0.3, 1.5, 1.0, 0.6), (0.1, 0.5, 0.0, -0.5, 1.0)],    # all three planes
            [(0.2, 0.1, 2.0, 1.0, 0.2), (0.4, -0.3, 1.5, 1.0, 0.6), (0.1, 0.5, 3.0, 1.0, 1.0)]]      # all beyond far: nothing
    for _ in range(n):
        tris.append([(rng.uniform(-2, 2), rng.uniform(-2, 2), rng.uniform(-3, 3), rng.choice([1, 1, 2, 0.5, -0.5, 0.004]), rng.uniform(0, 1))
                     for _ in range(3)])
    # the guard band (planes 3..6): the test viewport is 640 x 480, so its NDC bounds are about -7.4 .. 5.4 in x, -9.5 .. 7.5 in y
    gb = guard_consts([enc(640 * 8), enc(640 * 8), enc(480 * 8), enc(480 * 8)])
    tris += [[(0.2, 0.1, 0.0, 1.0, 0.5), (9.0, -0.3, 0.0, 1.0, 1.0), (0.1, 0.5, 0.0, 1.0, 1.5)],          # one vertex right of the band
             [(0.2, -12.0, 0.0, 1.0, 0.5), (0.4, 0.3, 0.0, 1.0, 1.0), (0.1, 0.5, -0.5, 0.5, 1.5)],        # one below (y), small w
             [(-9.0, 0.1, 0.0, 1.0, 0.5), (9.0, 0.3, 0.0, 1.0, 1.0), (0.1, 12.0, 0.0, 1.0, 1.5)],         # three sides: a hexagon
             [(0.2, 0.1, -1.5, 1.0, 0.2), (30.0, -0.3, 0.5, 1.0, 0.6), (0.1, 0.5, 0.0, -0.5, 1.0)],      # near + x + w planes
             [(20.0, 20.0, 0.0, 1.0, 0.2), (21.0, 20.0, 0.0, 1.0, 0.6), (20.0, 21.0, 0.0, 1.0, 1.0)]]     # entirely outside: nothing
    for _ in range(n):
        tris.append([(rng.uniform(-12, 12), rng.uniform(-12, 12), rng.uniform(-3, 3), rng.choice([1, 1, 2, 0.5, 0.25]), rng.uniform(0, 1))
                     for _ in range(3)])
    cases = [([mkv(*v) for v in tri]) for tri in tris]
    with open(f"{GEN}/clipnear_vectors{'_z0' if z0 else ''}.hex", 'w') as fh:
        fh.write(f"{nvar}\n{len(cases)}\n" + " ".join(f"{q:0{hw}x}" for q in gb) + "\n")
        for V in cases:
            out = clip(V, gb, z0)
            for v in V: fh.write(" ".join(f"{q:0{hw}x}" for q in v) + "\n")
            fh.write(f"{len(out)}\n")
            for t3 in out:
                for v in t3: fh.write(" ".join(f"{q:0{hw}x}" for q in v) + "\n")
    print(f"clipnear_vectors{'_z0' if z0 else ''}.hex: {len(cases)} triangles, nvar {nvar}")


def main():
    a = sys.argv[1:]
    if '--check' in a: sys.exit(0 if check_dz_grad() else 1)
    if '--clipvec' in a: clip_vectors(); clip_vectors(z0=1); return
    seed, mode, nv, nidx, indexed, u8, first, ioff, types, far = parse_args(a)
    d = build_draw(seed, mode, nv, nidx, indexed, u8, first, ioff, types, far=far, proj='--noproj' not in a)
    rng = random.Random(seed * 31 + 7)
    wid = int(a[a.index('--wid') + 1]) if '--wid' in a else 640
    hgt = int(a[a.index('--hgt') + 1]) if '--hgt' in a else 480
    dn, df = (F(a[a.index('--depth') + 1]), F(a[a.index('--depth') + 2])) if '--depth' in a else (F(0), F(1))   # glDepthRangef(n, f)
    st = dict(vp=[enc(wid * 8), enc(wid * 8), enc(hgt * 8), enc(hgt * 8),   # viewport in 1/16 pixel: ndc * w/2 * 16 + w/2 * 16
                  enc((df - dn) / 2), enc((dn + df) / 2)],                 # window z = ndc_z * (f - n) / 2 + (n + f) / 2
              cull=int(a[a.index('--cull') + 1]) if '--cull' in a else rng.choice([0, 0, 1, 2]),
              front_cw=1 if '--cw' in a else rng.randrange(2), draw_id=rng.randrange(1, 4096), wid=wid, hgt=hgt,
              fsmask=int(a[a.index('--fsmask') + 1], 16) if '--fsmask' in a else rng.choice([d['varmask'], d['varmask'], d['varmask'] & 0x25, 0x01]))
    st['lw'] = int(a[a.index('--lw') + 1]) if '--lw' in a else rng.choice([1, 1, 2, 3, 5])     # the line width (after the other state: earlier seeds unchanged)
    st['z0'] = int('--z0' in a)
    if st['z0']: st['vp'][4:6] = [enc(df - dn), enc(dn)]                                        # ndc_z in 0 .. 1: window z = n + ndc_z (f - n)
    st['gb'] = guard_consts(st['vp'])                                                          # the guard band's NDC bounds (VS_STATE words 9..12)
    # --sc x0 y0 x1 y1: the draw's rectangle, its scissor box (VS_STATE words 13 / 14: the 15-word form); without
    # it the command keeps its 13 words and board A takes the whole target. --scalt with --ndraw: the draws alternate, even
    # ones through the rectangle, odd ones without (both forms of the command, and the per-draw state queue)
    if '--sc' in a:
        k = a.index('--sc')
        st['sc'] = tuple(int(v) for v in a[k + 1:k + 5])
    # board A's coarse depth: --zc N = the draw's flags {INV 8, STRICT 4, CULL 2, OCC 1} (VS_STATE word 4 [28:25], the
    # records' header word 2 [15:13]); --zcd a,b,.. one per draw of --ndraw (cyclic); --lazy = VS_STATE word 4 bit 29 (TRI
    # items only in bands where a quad follows); --cz 'd16 ds cden csc cdepth' = the frame's depth as its FLUSH gives it
    # (as_<seed>_cz.hex with the lazy bit: a_raster_ref's culled stream, tb_a_render +CZ, b_ref aframe, whose ROP state then
    # keeps the promise the flags make)
    st['zc'] = int(a[a.index('--zc') + 1], 0) if '--zc' in a else 0
    zcd = [int(v, 0) for v in a[a.index('--zcd') + 1].split(',')] if '--zcd' in a else None
    if zcd: st['zc'] = zcd[0]
    st['lazy'] = int('--lazy' in a)
    # --ndraw N (draws overlap on A): the same draw N times back to back, each with its own MVP rows
    # (uniforms 0..3), draw id and cull mode — the record ids continue across the draws (one frame), the command stream
    # carries VS_UNIFORMS / VS_STATE / DRAW per draw; as_<seed>_state.mem / av_<seed>_expect.mem describe draw 0 alone
    ndraw = int(a[a.index('--ndraw') + 1]) if '--ndraw' in a else 1
    draws = [(d['params'], st)]
    drng = random.Random(seed * 101 + 5); fb = a_vtx_ref.fb
    for k in range(1, ndraw):
        pk = [[fb(0.45 + drng.uniform(-0.1, 0.1)), fb(drng.uniform(-0.1, 0.1)), fb(drng.uniform(-0.1, 0.1)), fb(drng.uniform(-0.2, 0.2))],
              [fb(drng.uniform(-0.1, 0.1)), fb(0.45 + drng.uniform(-0.1, 0.1)), fb(drng.uniform(-0.1, 0.1)), fb(drng.uniform(-0.2, 0.2))],
              [fb(drng.uniform(-0.1, 0.1)), fb(drng.uniform(-0.1, 0.1)), fb(0.4), fb(drng.uniform(-0.2, 0.2))],
              [fb(drng.uniform(-0.15, 0.15)), fb(drng.uniform(-0.15, 0.15)), fb(drng.uniform(-0.45, 0.3)), fb(1.0)]] + d['params'][4:]
        sk = dict(st); sk['draw_id'] = (st['draw_id'] + k) & 0x1FFF; sk['cull'] = drng.choice([0, 1, 2]); sk['front_cw'] = drng.randrange(2)
        if '--scalt' in a and (k & 1): sk.pop('sc', None)
        if zcd: sk['zc'] = zcd[k % len(zcd)]
        draws.append((pk, sk))
    recs = []; tid = 0; nclip = ndrop = 0
    for pk, sk in draws:
        res = d['res'] if pk is d['params'] else a_vtx_ref.run_vs(d['words'], d['attrs'], pk, d['uniq'], d['varmask'])
        for p in d['prims']:
            tri = []; psz = []
            for k in p:
                pos, var, ps = res[d['idx'][k]]
                tri.append(list(pos) + [var[i][c] for i in range(8) for c in range(4)]); psz.append(ps)
            if len(p) == 1: parts = [None]
            elif len(p) == 2: parts = [None]
            else:
                parts = clip(tri, sk['gb'], sk.get('z0', 0))
                if len(parts) != 1: nclip += 1
            for cv in parts:
                r = point_setup(tri[0], psz[0], sk, tid) if len(p) == 1 else line_setup(tri[0], tri[1], sk, tid) if len(p) == 2 else tri_setup(cv, sk, tid)
                if os.environ.get('A_SETUP_DEBUG'):
                    ws = [fr.to_real(v[3], FMT) for v in (cv if cv is not None else tri)]
                    zs_ = [fr.to_real(v[2], FMT) for v in (cv if cv is not None else tri)]
                    print(f"  prim {p} parts {len(parts)}: w {[float(w) if w is not None else None for w in ws]} z {[float(z) if z is not None else None for z in zs_]} -> "
                          f"{'dropped' if r is None else f'record {tid} bands {r[0]}..{r[1]}'}")
                if r is None: ndrop += 1; continue
                recs.append((tid,) + r); tid += 1
    tag = f"as_{seed}"
    with open(f"{GEN}/{tag}_state.mem", 'w') as f:                # {sy1, sx1, sy0, sx0 (12 bits each: draw 0's rectangle), gyhi, gylo, gxhi, gxlo, zo, zs, z0, lw[6:0], fsmask[7:0], hgt[11:0], wid[11:0], draw_id[12:0], front_cw, cull[1:0], syo, sys, sxo, sxs}
        rc = rect_of(st)
        v = (rc[3] << 332) | (rc[2] << 320) | (rc[1] << 308) | (rc[0] << 296) | \
            (st['gb'][3] << 272) | (st['gb'][2] << 248) | (st['gb'][1] << 224) | (st['gb'][0] << 200) | \
            (st['vp'][5] << 176) | (st['vp'][4] << 152) | \
            (st['z0'] << 151) | (st['lw'] << 144) | (st['fsmask'] << 136) | (hgt << 124) | (wid << 112) | (st['draw_id'] << 99) | (st['front_cw'] << 98) | (st['cull'] << 96) | \
            (st['vp'][3] << 72) | (st['vp'][2] << 48) | (st['vp'][1] << 24) | st['vp'][0]
        f.write(f"{v:086x}\n")                                       # 344 bits
    with open(f"{GEN}/{tag}_rec.hex", 'w') as f:
        for tid, lo, hi, words in recs:
            f.write(f"{tid:x} {lo:x} {hi:x} {len(words):x} " + " ".join(f"{w:08x}" for w in words) + "\n")
    write_cmds(tag, d, draws)
    czp = f"{GEN}/{tag}_cz.hex"
    if '--cz' in a: open(czp, 'w').write(" ".join(str(int(v, 0)) for v in a[a.index('--cz') + 1].split()) + f" {st['lazy']}\n")   # decimal
    elif os.path.exists(czp): os.remove(czp)
    print(f"{tag}: {len(d['prims'])} primitives (mode {d['mode']}) x {ndraw} draw(s) -> {len(recs)} records ({nclip} clipped to 0 / 2, {ndrop} dropped: degenerate / "
          f"culled / off-screen); cull {st['cull']} front_cw {st['front_cw']} fsmask {st['fsmask']:02x} lw {st['lw']} -> tools/gen/{tag}_*")


def write_cmds(tag, d, draws):
    """The same draw(s) as host commands for board A (fb_dispatch_a), as ingress-queue entries: a header
    burst {0x51, type, body bytes} | seq | 0.. then the body words in bursts -> tools/gen/as_<seed>_cmds.hex, one
    256-bit burst per line. FRAME_BEGIN, CREATE_BUFFER x4 (handles 1 VBO A, 2 VBO B, 3 indices, 4 program), VS_PROGRAM,
    VERTEX_ATTR x16, then per draw VS_UNIFORMS (the 4 MVP rows), VS_STATE, DRAW; FENCE."""
    T = dict(CREATE_BUFFER=0x10, FENCE=0x40, VS_PROGRAM=0x51, VS_UNIFORMS=0x52, VERTEX_ATTR=0x53, VS_STATE=0x54, DRAW=0x55, FRAME_BEGIN=0x56)
    bursts = []; seq = [0]
    def cmd(t, body):
        seq[0] += 1
        bursts.append([(0x51 << 24) | (T[t] << 16) | (4 * len(body)), seq[0], 0, 0, 0, 0, 0, 0])
        for k in range(0, len(body), 8): bursts.append(body[k:k + 8] + [0] * (8 - len(body[k:k + 8])))
    b = d['bases']
    cmd('FRAME_BEGIN', [0])
    for h, base, size in ((1, b['a'], 0x2000), (2, b['b'], 0x1000), (3, b['idx'], 0x1000), (4, b['prog'], 0x1000)):
        cmd('CREATE_BUFFER', [h, 4 * base, 4 * size])                  # {usage 0, handle}, base, size in bytes
    cmd('VS_PROGRAM', [4, 0, len(d['words'])])
    for i, (en, base, size, ty, nm, stride, off) in enumerate(d['desc']):        # base / stride / offset in bytes; type + normalized
        h = 0 if not en else (1 if base == 4 * b['a'] else 2)
        cmd('VERTEX_ATTR', [i | (en << 8) | (size << 16) | (ty << 20) | (nm << 24), h, stride, off] + list(d['consts'][i]))
    for params, st in draws:
        cmd('VS_UNIFORMS', [(4 << 16) | 0] + [c for i in range(4) for c in params[i]])
        cmd('VS_STATE', st['vp'][:4] + [st['cull'] | (st['front_cw'] << 8) | (st['lw'] << 9) | (st['fsmask'] << 16) | (st.get('z0', 0) << 24) |
                                        (st.get('zc', 0) << 25) | (st.get('lazy', 0) << 29), st['wid'] | (st['hgt'] << 16), st['draw_id']]
                        + st['vp'][4:] + st['gb']                              # words 7, 8: the depth range zs, zo; 9..12: the guard band
                        + ([st['sc'][0] | (st['sc'][1] << 16), st['sc'][2] | (st['sc'][3] << 16)] if 'sc' in st else []))   # 13, 14: the rectangle, when the draw has one
        cmd('DRAW', [d['mode'] | (int(d['indexed']) << 8) | (int(d['u8']) << 16), 3, d.get('ioff_bytes', 0), d['first'], d['nidx']])
    cmd('FENCE', [0x5EED])
    with open(f"{GEN}/{tag}_cmds.hex", 'w') as f:
        for bw in bursts: f.write(''.join(f"{bw[7 - k]:08x}" for k in range(8)) + "\n")


if __name__ == '__main__': main()
