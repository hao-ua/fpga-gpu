#!/usr/bin/env python3
"""a_raster_ref — model of board A's render side (src/rtl/a_render.sv): the render stream A
sends B for one frame, from the triangle records and the bin lists of a draw.

Input: tools/gen/as_<seed>_rec.hex (a_setup_ref: per record "id band_lo band_hi nwords w0 w1 ..." — the raster header
burst, then the TRI item) and as_<seed>_state.mem (the render target's width / height). The band walk (a_render):
    for band b = 0 .. ceil(hgt / NL) - 1:  BAND {y0 = NL b}
        for each triangle of band b's bin list (push order = record id order; a triangle is in bands lo..hi):
            its TRI item when b == band_lo (the first band it touches) or when it is not the current one;
            every quad (2 x 2 pixels) of the triangle's bounding box inside the band and the target with a covered
            pixel, quad rows then quad columns ascending: QUAD {cov, qy, qx} — ONE beat: B forms
            the quad's edge values from the TRI item's reference values (the edge values at the centre of pixel (0, 0),
            >> s, inserted by the walk after the record's 11th beat: words w22..w24, then a 0) and the TRI's steps
    FRAME_END
Bands are 8 rows for targets up to 1024 wide, else 4 (b_ref.band_lines; a_setup bins with the same rule).
Coverage: pixel centres with the tie rule (b_ref.pixel_covered), on the exact (unshifted) edge values, pixels inside
the target (the same formulas as b_ref.pixel_edges, stepped incrementally here).
Output: tools/gen/as_<seed>_stream.hex, one 64-bit beat per line.
    python3 tools/a_raster_ref.py <seed>
"""
import os, sys
from fractions import Fraction as F
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import b_ref

GEN = b_ref.GEN
NL = 4                                                            # set per target by stream() / check_vs_b_ref (b_ref.band_lines)


def s16(v): return v - 0x10000 if v & 0x8000 else v
def s32(v): return v - 0x100000000 if v & 0x80000000 else v


def load_records(path):
    recs = []
    for ln in open(path):
        t = ln.split()
        if not t: continue
        tid, lo, hi, n = (int(v, 16) for v in t[:4])
        w = [int(v, 16) for v in t[4:4 + n]]
        recs.append(dict(id=tid, lo=lo, hi=hi, w=w, s=(w[0] >> 16) & 3, prim=(w[0] >> 26) & 3, lw=(w[0] >> 18) & 0x7F, ps=w[3],
                         U=[(s16(w[4 + k] & 0xFFFF), s16(w[4 + k] >> 16)) for k in range(3)],
                         U18=(s32(w[5]), s32(w[6])),         # a point's centre before the 16-bit saturation (words 5 / 6)
                         # the rectangle [x0, x1) x [y0, y1) in the header's spare bits (a_setup_ref hdr_rect): the
                         # draw's scissor box cut to the target; the walk's bounds
                         R=(w[1] >> 20, w[2] >> 20, ((w[7] >> 26) << 6) | ((w[7] >> 10) & 0x3F), ((w[0] & 0xFF) << 4) | ((w[2] >> 16) & 0xF))))
    return recs


M32 = 0xFFFFFFFF


def point_quads(U0, ps, y0, R):
    """A point: the square [xl, xl + ps) x [yb, yb + ps) around the centre (1/16 pixel units, half = ps >> 1); a
    pixel centre is covered iff 0 <= cx - xl < ps and 0 < yt - cy <= ps (yt = yb + ps); e = (ps, cx - xl, yt - cy) at
    the quad's pixel 0 (s = 0: a point is < 2^11 units wide)"""
    cx0, cy0 = U0
    half = ps >> 1
    xl, yb = cx0 - half, cy0 - half; yt = yb + ps
    rx0, ry0, rx1, ry1 = R
    xmin, xmax = max(rx0, xl >> 4), min(rx1 - 1, (xl + ps - 1) >> 4)
    ymin, ymax = max(ry0, yb >> 4), min(ry1 - 1, (yb + ps - 1) >> 4)
    out = []
    if xmin > xmax or ymin > ymax: return out
    for qy in range(y0 // 2, (y0 + NL) // 2):
        py = 2 * qy
        if py + 1 < ymin or py > ymax: continue
        for qx in range((xmin & ~1) // 2, xmax // 2 + 1):
            cov = 0
            for p in range(4):
                px, pyp = 2 * qx + (p & 1), py + (p >> 1)
                if not (rx0 <= px < rx1 and ry0 <= pyp < ry1): continue
                e1, e2 = 16 * px + 8 - xl, yt - (16 * pyp + 8)
                if 0 <= e1 < ps and 0 < e2 <= ps: cov |= 1 << p
            if cov:
                out.append([(1 << 28) | (cov << 24) | (qy << 12) | qx])
    return out


def ref_words(r):
    """the TRI item's reference edge values w22..w24 — the edge values at the centre of pixel (0, 0), (8, 8) in 1/16
    pixel, >> s, as 32-bit words — for the record's kind: a triangle's three edges, a line's (len2 - e1, e1, 0), a point's
    (ps, cx - xl, yt - cy); a_render computes them in its W_REF stages from the record's header"""
    s = r['s']
    if r['prim'] == 2:
        cx0, cy0 = r['U18']; half = r['ps'] >> 1
        xl, yb = cx0 - half, cy0 - half; yt = yb + r['ps']
        e = [r['ps'], 8 - xl, yt - 8]
    elif r['prim'] == 1:
        (xa, ya), (xb, yb) = r['U'][0], r['U'][1]
        dx, dy = xb - xa, yb - ya
        e1 = (8 - xa) * dx + (8 - ya) * dy
        e = [dx * dx + dy * dy - e1, e1, 0]
    else:
        (x0, y0u), (x1, y1u), (x2, y2u) = r['U']
        dx1, dy1, dx2, dy2 = x1 - x0, y1u - y0u, x2 - x0, y2u - y0u
        e1 = (8 - x0) * dy2 - dx2 * (8 - y0u)
        e2 = dx1 * (8 - y0u) - (8 - x0) * dy1
        e = [dx1 * dy2 - dx2 * dy1 - e1 - e2, e1, e2]
    return [(v >> s) & M32 for v in e]


def line_quads(Ua, Ub, lw, s, y0, R):
    """A line segment a -> b of width lw (ES 2.0 §3.4.1 / §3.4.2, the Bresenham form of the diamond-exit rule):
    x-major (|dx| >= |dy|): one fragment per column whose centre cx lies in [xa, xb) (dx > 0) or (xb, xa] (dx < 0), in
    the row floor((y(cx) - 8 (lw - 1)) / 16) with y(cx) = ya + (cx - xa) dy / dx exact, replicated over lw rows;
    y-major the same with x and y exchanged. Quads: per column pair (x-major) the union of the two columns' rows,
    quads in row order; per row pair (y-major) the union of the two rows' columns. e1 = (c - a) . (b - a) at the quad's
    pixel 0, e0 = len2 - e1, e2 = 0, all >> s."""
    (xa, ya), (xb, yb) = Ua, Ub
    dx, dy = xb - xa, yb - ya
    len2 = dx * dx + dy * dy
    out = []
    rx0, ry0, rx1, ry1 = R
    ylo, yhi = max(ry0, y0), min(ry1 - 1, y0 + NL - 1)             # the band's rows in the rectangle
    def quad(qx, qy, cov):
        return [(1 << 28) | (cov << 24) | (qy << 12) | qx]
    if abs(dx) >= abs(dy):
        lo, hi = ((xa + 7) >> 4, (xb - 9) >> 4) if dx > 0 else ((xb + 8) >> 4, (xa - 8) >> 4)
        lo, hi = max(lo, rx0), min(hi, rx1 - 1)
        if lo > hi: return out
        den = 16 * abs(dx)
        def row0(px):
            num = (ya - 8 * (lw - 1)) * dx + (16 * px + 8 - xa) * dy
            return (-num if dx < 0 else num) // den
        for qx in range(lo // 2, hi // 2 + 1):
            cols = {c: row0(c) for c in (2 * qx, 2 * qx + 1) if lo <= c <= hi}
            rmin, rmax = min(cols.values()), max(cols.values()) + lw - 1
            for qy in range(max(rmin >> 1, ylo >> 1), min(rmax >> 1, yhi >> 1) + 1):
                cov = 0
                for p in range(4):
                    px, py = 2 * qx + (p & 1), 2 * qy + (p >> 1)
                    if px in cols and cols[px] <= py < cols[px] + lw and rx0 <= px < rx1 and ry0 <= py < ry1: cov |= 1 << p
                if cov: out.append(quad(qx, qy, cov))
    else:
        lo, hi = ((ya + 7) >> 4, (yb - 9) >> 4) if dy > 0 else ((yb + 8) >> 4, (ya - 8) >> 4)
        lo, hi = max(lo, ylo), min(hi, yhi)
        if lo > hi: return out
        den = 16 * abs(dy)
        def col0(py):
            num = (xa - 8 * (lw - 1)) * dy + (16 * py + 8 - ya) * dx
            return (-num if dy < 0 else num) // den
        for qy in range(lo // 2, hi // 2 + 1):
            rws = {r: col0(r) for r in (2 * qy, 2 * qy + 1) if lo <= r <= hi}
            cmin, cmax = min(rws.values()), max(rws.values()) + lw - 1
            for qx in range(max(cmin >> 1, rx0 >> 1), min(cmax >> 1, (rx1 - 1) >> 1) + 1):
                cov = 0
                for p in range(4):
                    px, py = 2 * qx + (p & 1), 2 * qy + (p >> 1)
                    if py in rws and rws[py] <= px < rws[py] + lw and rx0 <= px < rx1 and ry0 <= py < ry1: cov |= 1 << p
                if cov: out.append(quad(qx, qy, cov))
    return out


def tri_quads(U, s, y0, R):
    """the QUAD words of one triangle (window positions U in 1/16 pixel) in band y0 .. y0 + NL - 1, in scan order"""
    (x0, y0u), (x1, y1u), (x2, y2u) = U
    dx1, dy1, dx2, dy2 = x1 - x0, y1u - y0u, x2 - x0, y2u - y0u
    a2 = dx1 * dy2 - dx2 * dy1
    front = a2 > 0
    A1, B1, A2, B2 = 16 * dy2, -16 * dx2, -16 * dy1, 16 * dx1
    AB = [-A1 - A2, -B1 - B2, A1, B1, A2, B2]
    # the bounding box in pixels, clamped to the rectangle (a centre 16 px + 8 can only be inside for px in this range)
    rx0, ry0, rx1, ry1 = R
    xmin, xmax = max(rx0, min(x0, x1, x2) >> 4), min(rx1 - 1, max(x0, x1, x2) >> 4)
    ymin, ymax = max(ry0, min(y0u, y1u, y2u) >> 4), min(ry1 - 1, max(y0u, y1u, y2u) >> 4)
    out = []
    if xmin > xmax or ymin > ymax: return out
    qx0 = xmin & ~1
    for qy in range(y0 // 2, (y0 + NL) // 2):
        py = 2 * qy
        if py + 1 < ymin or py > ymax: continue
        cy = 16 * py + 8
        # edge values at the centre of pixel (qx0 * 2, py); then + 2 A per quad along x
        cx = 16 * qx0 + 8
        e1 = (cx - x0) * dy2 - dx2 * (cy - y0u)
        e2 = dx1 * (cy - y0u) - (cx - x0) * dy1
        e = [a2 - e1 - e2, e1, e2]
        for qx in range(qx0 // 2, xmax // 2 + 1):
            cov = 0
            for p in range(4):
                px, pyp = 2 * qx + (p & 1), py + (p >> 1)
                if not (rx0 <= px < rx1 and ry0 <= pyp < ry1): continue
                ep = [e[k] + AB[2 * k] * (p & 1) + AB[2 * k + 1] * (p >> 1) for k in range(3)]
                if b_ref.pixel_covered(ep, AB, front): cov |= 1 << p
            if cov:
                out.append([(1 << 28) | (cov << 24) | (qy << 12) | qx])
            e = [e[k] + 2 * AB[2 * k] for k in range(3)]
    return out


def stream(recs, wid, hgt):
    global NL
    NL = b_ref.band_lines(wid)
    beats = []
    def emit(words):
        words = words + [0] * (len(words) & 1)
        for k in range(0, len(words), 2): beats.append(words[k] | words[k + 1] << 32)
    nbands = (hgt + NL - 1) // NL
    cur = None
    nq = 0
    for b in range(nbands):
        y0 = NL * b
        emit([(4 << 28) | y0, 0])
        for r in recs:                                        # the bin list of band b, oldest first
            if not (r['lo'] <= b <= r['hi']): continue
            # the whole TRI item again (B never re-reads a record from its DDR); the walk inserts the reference beats after
            # the record's 11th beat (w22..w24 + 0), before the varyings
            if b == r['lo'] or cur != r['id']: emit(r['w'][8:30] + ref_words(r) + [0] + r['w'][30:])
            cur = r['id']
            qs = (point_quads(r['U18'], r['ps'], y0, r['R']) if r['prim'] == 2 else
                  line_quads(r['U'][0], r['U'][1], r['lw'], r['s'], y0, r['R']) if r['prim'] == 1 else
                  tri_quads(r['U'], r['s'], y0, r['R']))
            for q in qs:
                nq += 1
                emit(q)
    emit([5 << 28, 0])
    return beats, nq


def check_vs_b_ref(n=200):
    """tri_quads against b_ref.quad_cover on random triangles (the two coverage formulations agree), and ref_words against
    b_ref.ref_edges (the reference edge values the TRI item carries)"""
    import random
    global NL
    rng = random.Random(5); bad = 0
    wid, hgt = 64, 32
    NL = b_ref.band_lines(wid)
    for _ in range(n):
        P, a2, z, q, zs, ws = b_ref.rand_tri(rng)
        U = b_ref.units16(P); s = b_ref.edge_shift(int(a2 * 256))
        if ref_words(dict(prim=0, s=s, U=U)) != b_ref.ref_edges(P, s): bad += 1
        for b in range(hgt // NL):
            got = tri_quads(U, s, NL * b, (0, 0, wid, hgt))
            exp = []
            for qy in range(NL * b // 2, (NL * b + NL) // 2):
                for qx in range(wid // 2):
                    cov = b_ref.quad_cover(P, qx, qy, wid, hgt)
                    if cov: exp.append([(1 << 28) | (cov << 24) | (qy << 12) | qx])
            bad += got != exp
    print("a_raster_ref vs b_ref:", "ok" if bad == 0 else f"{bad} band(s) differ")
    return bad == 0


def check_rect(n=300):
    """The rectangle changes nothing board B can show: for random triangles, points and lines and random
    rectangles, the quads walked inside the rectangle are exactly the quads of the whole target with the pixels outside
    the rectangle taken out of their coverage (what B's scissor test does to them) and the quads left empty dropped —
    the same quads, in the same order."""
    import random
    global NL
    rng = random.Random(11); bad = 0; nq = 0
    wid, hgt = 64, 32
    NL = b_ref.band_lines(wid)
    full = (0, 0, wid, hgt)
    def masked(qs, R):
        out = []
        for (w,) in qs:
            qx, qy, cov = w & 0xFFF, (w >> 12) & 0xFFF, (w >> 24) & 0xF
            c2 = 0
            for p in range(4):
                px, py = 2 * qx + (p & 1), 2 * qy + (p >> 1)
                if (cov >> p) & 1 and R[0] <= px < R[2] and R[1] <= py < R[3]: c2 |= 1 << p
            if c2: out.append([(1 << 28) | (c2 << 24) | (qy << 12) | qx])
        return out
    for i in range(n):
        x0, y0 = rng.randrange(wid), rng.randrange(hgt)
        R = (x0, y0, rng.randrange(x0 + 1, wid + 1), rng.randrange(y0 + 1, hgt + 1))
        if i % 7 == 0: R = (x0, y0, x0 + 1, y0 + 1)                    # one pixel
        if i % 11 == 0: R = full
        kind = i % 3
        if kind == 0:
            P, a2, z, q, zs, ws = b_ref.rand_tri(rng)
            U = b_ref.units16(P); s = b_ref.edge_shift(int(a2 * 256))
            f = lambda yb, Rr: tri_quads(U, s, yb, Rr)
        elif kind == 1:
            ps = 2 * rng.randrange(8, 16 * 12); U0 = (rng.randrange(-200, 16 * wid + 200), rng.randrange(-200, 16 * hgt + 200))
            f = lambda yb, Rr: point_quads(U0, ps, yb, Rr)
        else:
            Ua = (rng.randrange(-300, 16 * wid + 300), rng.randrange(-300, 16 * hgt + 300))
            Ub = (rng.randrange(-300, 16 * wid + 300), rng.randrange(-300, 16 * hgt + 300))
            if Ua == Ub: continue
            lw = rng.choice([1, 1, 2, 3, 5])
            f = lambda yb, Rr: line_quads(Ua, Ub, lw, 0, yb, Rr)
        for b in range(hgt // NL):
            # a band the rectangle's rows do not reach is never visited (a_setup bins by the rectangle): nothing to compare
            if not (max(R[1], NL * b) <= min(R[3] - 1, NL * b + NL - 1)): continue
            got = f(NL * b, R); exp = masked(f(NL * b, full), R)
            nq += len(got)
            if got != exp:
                bad += 1
                if bad <= 5: print(f"  kind {kind} rect {R} band {b}: {len(got)} quads, expected {len(exp)}")
    print(f"a_raster_ref rectangle = the target's walk masked ({nq} quads):", "ok" if bad == 0 else f"{bad} band(s) differ")
    return bad == 0


def main():
    if '--check' in sys.argv: sys.exit(0 if (check_vs_b_ref() and check_rect()) else 1)
    seed = sys.argv[1] if len(sys.argv) > 1 else "1"
    recs = load_records(f"{GEN}/as_{seed}_rec.hex")
    st = int(open(f"{GEN}/as_{seed}_state.mem").read().split()[0], 16)
    wid, hgt = (st >> 112) & 0xFFF, (st >> 124) & 0xFFF
    beats, nq = stream(recs, wid, hgt)
    with open(f"{GEN}/as_{seed}_stream.hex", 'w') as f:
        for b in beats: f.write(f"{b:016x}\n")
    print(f"as_{seed}: {len(recs)} records, {wid} x {hgt}, {(hgt + NL - 1) // NL} bands -> {nq} quads, {len(beats)} beats"
          f" -> tools/gen/as_{seed}_stream.hex")


if __name__ == '__main__': main()
