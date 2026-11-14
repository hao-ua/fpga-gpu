#!/usr/bin/env python3
"""fb_tex3d_demo — 3D textures on the monitor: a 128 x 128 x 128 volume of two interlocked glass rings with bright cores,
turning. Volume rendering: slices through the volume blended back to front, every texel on the screen from the TMU's 3D
path (a board B built with 3D textures). The last frame stays on the screen when the run ends.

Full stacks of slices cost 925 k quads a frame and held the core 99.9 % busy at 8 fps, with the rings in 15 % of the
volume. So the slices stay in the volume's own coordinates, a stack per axis, and hold only the rectangles of cells near
the rings; each frame draws the stack most nearly facing the viewer, back to front.

  python3 host/fb/fb_tex3d_demo.py [--ip 192.168.1.10] [--mode 0] [--secs 120] [--slices 36] [--size 470] [--png FILE]

--size: the object's bounding circle in pixels. --png: the last frame from B's DDR. Standard library + the repository's tools.
"""
import collections, math, os, struct, sys, time
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(HERE, '..', '..', 'tools'))
import fbproto as fp
import glslc, isa64
from fb_image import MODE_SIZE
from fb_shader import (words64_bytes, f24, H_VBO, H_IBO, H_VSPROG, A_VBO, A_IBO, A_VSPROG, H_FSPROG, B_FSPROG,
                       H_IMG, B_IMG, DRAW_ID, err_text)
from fb_texperf import B_TEX
from fb_tex3d import Volume, RGBA8, BILINEAR, CLAMP, H_VOL, write_png

TESTS = os.path.join(HERE, '..', '..', 'tests')
CLEAR = (0.02, 0.03, 0.09, 1.0)
SRC_ALPHA, ONE_MINUS_SRC_ALPHA, ONE = 6, 7, 1                   # blend factors, h4
R_MAJOR, R_TUBE, R_CORE = 0.46, 0.17, 0.085                     # the rings: radius, glass, bright core
OFFSET = 0.27                                                   # their centres on x
LIGHT = (0.42, 0.60, 0.68)
CELLS = 96                                                      # the cells of a slice, per side
MARGIN = 0.028 + 1.42 / CELLS                                   # a cell is kept this near the tube: its half diagonal + the alpha ramp + a voxel
RING_A, RING_B = ((-OFFSET, 0.0, 0.0), 2), ((OFFSET, 0.0, 0.0), 1)   # centre, axis


def ring(p, c, axis):
    """distance of p from the ring's tube centre; n = the unit vector from it; phi = the angle around the ring"""
    q = [p[i] - c[i] for i in range(3)]
    u, v = [i for i in range(3) if i != axis]
    a = math.hypot(q[u], q[v])
    if a < 1e-6: return 9.0, (0.0, 0.0, 0.0), 0.0
    k = (a - R_MAJOR) / a
    e = [0.0, 0.0, 0.0]; e[u] = q[u] * k; e[v] = q[v] * k; e[axis] = q[axis]
    d = math.sqrt(e[0] * e[0] + e[1] * e[1] + e[2] * e[2])
    if d < 1e-6: return 0.0, (0.0, 0.0, 1.0), math.atan2(q[v], q[u])
    return d, (e[0] / d, e[1] / d, e[2] / d), math.atan2(q[v], q[u])


def smooth(a, b, x):
    t = max(0.0, min(1.0, (x - a) / (b - a)))
    return t * t * (3.0 - 2.0 * t)


def shade(d, n, phi, glass, core, opacity):
    """one ring's voxel: a dark band every other 30 degrees shows it turn; (r, g, b, a) in 0..1. opacity: the alpha of a
    slice is this power of the 32-slice alpha, so the object looks the same with any number of slices"""
    lit = 0.70 + 0.30 * max(0.0, n[0] * LIGHT[0] + n[1] * LIGHT[1] + n[2] * LIGHT[2])
    band = 0.62 if int((phi + math.pi) * 6.0 / math.pi) & 1 else 1.0
    k = smooth(R_CORE + 0.02, R_CORE - 0.02, d)                 # 1 inside the core
    a = smooth(R_TUBE + 0.012, R_TUBE - 0.012, d) * (0.34 + 0.65 * k)
    return tuple(min(1.0, lit * (glass[c] + (core[c] * band - glass[c]) * k)) for c in range(3)) + (1.0 - (1.0 - a) ** opacity,)


def rings_slice_fn(opacity):
    def rings_slice(w, h, dep, z, l):
        buf = bytearray(w * h * 4)
        pz = (z + 0.5) / dep * 2.0 - 1.0
        o = 0
        for y in range(h):
            py = (y + 0.5) / h * 2.0 - 1.0
            for x in range(w):
                px = (x + 0.5) / w * 2.0 - 1.0
                p = (px, py, pz)
                d1, n1, f1 = ring(p, *RING_A)
                d2, n2, f2 = ring(p, *RING_B)
                if d1 < R_TUBE + 0.03 or d2 < R_TUBE + 0.03:
                    if d1 <= d2: c = shade(d1, n1, f1, (0.95, 0.30, 0.08), (1.0, 0.92, 0.45), opacity)
                    else:        c = shade(d2, n2, f2, (0.08, 0.40, 0.95), (0.55, 1.0, 1.0), opacity)
                    buf[o:o + 4] = bytes(min(255, int(v * 255.0 + 0.5)) for v in c)
                o += 4
        return buf
    return rings_slice


def near(p):
    """p is close enough to a ring for its cell to be drawn"""
    lim = R_TUBE + MARGIN
    return ring(p, *RING_A)[0] < lim or ring(p, *RING_B)[0] < lim


def rectangles(axis, c):
    """the cells near the rings on the slice at coordinate c of `axis`, as (u0, v0, u1, v1) in cells: rows of runs, a run
    going on into the next row while that row has the same one"""
    u, v = [i for i in range(3) if i != axis]
    p = [0.0, 0.0, 0.0]; p[axis] = c
    live = {}; out = []
    for j in range(CELLS + 1):
        runs = set()
        if j < CELLS:
            p[v] = (j + 0.5) * 2.0 / CELLS - 1.0
            i = 0
            while i < CELLS:
                p[u] = (i + 0.5) * 2.0 / CELLS - 1.0
                if near(p):
                    s = i
                    while i < CELLS:
                        p[u] = (i + 0.5) * 2.0 / CELLS - 1.0
                        if not near(p): break
                        i += 1
                    runs.add((s, i))
                else: i += 1
        for r in [r for r in live if r not in runs]:
            out.append((r[0], live.pop(r), r[1], j))
        for r in runs:
            live.setdefault(r, j)
    return out


def stacks(n):
    """per axis the slices in ascending order: (FP24 words, [[indices of slice k] for k], ...); the vertices of all three
    share one buffer. Returns the words and, per axis and direction, the index list back to front"""
    vw = []; ranges = []
    for axis in range(3):
        u, v = [i for i in range(3) if i != axis]
        per = []
        for k in range(n):
            c = -1.0 + (k + 0.5) * 2.0 / n
            cw = f24(c); il = []
            for u0, v0, u1, v1 in rectangles(axis, c):
                base = len(vw) // 3
                for a, b in ((u0, v0), (u1, v0), (u1, v1), (u0, v1)):
                    w = [0, 0, 0]; w[axis] = cw; w[u] = f24(a * 2.0 / CELLS - 1.0); w[v] = f24(b * 2.0 / CELLS - 1.0)
                    vw += w
                il += [base, base + 1, base + 2, base, base + 2, base + 3]
            per.append(il)
        ranges.append(per)
    return vw, ranges


def rotation(t):
    """volume -> view, rows: a steady turn about a tilted axis"""
    a = 0.75 * t; b = 0.31 * t + 0.5
    ca, sa, cb, sb = math.cos(a), math.sin(a), math.cos(b), math.sin(b)
    Ry = ((ca, 0.0, sa), (0.0, 1.0, 0.0), (-sa, 0.0, ca))
    Rx = ((1.0, 0.0, 0.0), (0.0, cb, -sb), (0.0, sb, cb))
    return tuple(tuple(sum(Ry[i][k] * Rx[k][j] for k in range(3)) for j in range(3)) for i in range(3))


def main():
    a = sys.argv[1:]
    def opt(name, default, conv=str): return conv(a[a.index(name) + 1]) if name in a else default
    ip = opt('--ip', fp.BOARD_IP); mode = opt('--mode', 0, int); secs = opt('--secs', 120.0, float)
    nslices = opt('--slices', 36, int); size = opt('--size', 470, int); png = opt('--png', None)
    rv = glslc.compile_stage(open(os.path.join(TESTS, 'tex3d_demo.vert'), encoding='utf-8').read(), 'vs', isa64=True, forward=True)
    rf = glslc.compile_stage(open(os.path.join(TESTS, 'tex3d_demo.frag'), encoding='utf-8').read(), 'fs', isa64=True, forward=True)
    wv, _ = isa64.assemble(rv['sasm'].splitlines(), vs=True)
    wf, _ = isa64.assemble(rf['sasm'].splitlines())
    vconst = [tuple(float(c) for c in k) for k in rv['const_pool']]
    fconst = [tuple(float(c) for c in k) for k in rf['const_pool']]
    print(f"programs: vertex {len(wv)}, fragment {len(wf)} instructions")
    t0 = time.monotonic()
    vol = Volume('rings', 128, 128, 128, RGBA8, 1, rings_slice_fn(32.0 / nslices))
    W, H = MODE_SIZE[mode]
    vw, ranges = stacks(nslices)
    # one index buffer: per axis the slices ascending, then descending
    idx = []; start = {}
    for axis in range(3):
        for up in (True, False):
            start[(axis, up)] = (len(idx) * 2, sum(len(s) for s in ranges[axis]))
            for s in (ranges[axis] if up else reversed(ranges[axis])): idx += s
    print(f"slices: {len(vw) // 3} vertices, {len(idx) // 6 // 3} triangles a frame on average, ready {time.monotonic() - t0:.1f} s after the start")
    bd = fp.Board(ip)
    bd.hello()
    st = bd.status()
    lfl = fp.decode_link_flags(st['link_flags'])
    if not (lfl['trained'] and lfl['b_alive']): raise fp.ProtoError("the board link is not up / B is not alive")
    bd.set_mode(mode)
    bd.create_buffer(H_VBO, A_VBO, 4 << 20); bd.write_buffer(H_VBO, 0, b''.join(struct.pack("<I", w) for w in vw))
    bd.create_buffer(H_IBO, A_IBO, 1 << 20); bd.write_buffer(H_IBO, 0, struct.pack("<%dH" % len(idx), *idx))
    bd.create_buffer(H_VSPROG, A_VSPROG, 4096); bd.write_buffer(H_VSPROG, 0, words64_bytes(wv))
    bd.vs_program(H_VSPROG, 0, len(wv))
    for i in range(16):
        if i == 0: bd.vertex_attr(0, True, 3, H_VBO, 12, 0, type=fp.AT_FP24)
        else: bd.vertex_attr(i, False, 4, 0, 0, 0, (0.0, 0.0, 0.0, 1.0))
    bd.vs_state((W * 8.0, W * 8.0, H * 8.0, H * 8.0), cull=0, front_cw=0, fsmask=0x01, width=W, height=H, draw_id=DRAW_ID)
    bd.create_buffer(H_FSPROG, B_FSPROG, 65536); bd.write_buffer(H_FSPROG, 0, words64_bytes(wf))
    for h, base in zip(H_IMG, B_IMG):
        bd.create_image(h, base, W, H, fp.FMT_RGBA8, origin=1, usage=fp.USE_RT | fp.USE_SCANOUT | fp.USE_TRANSFER)
    nb = vol.upload(bd, H_VOL, B_TEX)
    bd.fence(timeout=60.0)
    print(f"volume: {nb / 1e6:.1f} MB ready {time.monotonic() - t0:.1f} s after the start")
    uni = [(0.0, 0.0, 0.0, 0.0)] * rf['const_base'] + fconst
    h = [0] * 23
    h[0] = 1 << 6 | 0xF << 16                                  # blend, colour mask RGBA, no depth: the stack is in order
    h[4] = SRC_ALPHA | ONE_MINUS_SRC_ALPHA << 4 | ONE << 8 | ONE_MINUS_SRC_ALPHA << 12
    h[11] = len(wf) << 16 | H_FSPROG; h[12] = 0; h[13] = len(uni)
    h[15 + rf['samplers']['uVol']] = H_VOL | BILINEAR << 16 | CLAMP << 20 | CLAMP << 22 | CLAMP << 25
    bd.draw_state(DRAW_ID, h, uni)
    bd.fence(timeout=10.0)
    errs = bd.take_errors()
    for e in errs: print("  EV_ERROR seq %d: %s" % (e[0], err_text(e[:4])))
    if errs: return 1
    uv = rv['uniforms']; ub = rv['const_base']
    cc = tuple(fp.c16(c) for c in CLEAR)
    pending = collections.deque(); i = -1; t0 = time.monotonic(); tm = t0
    while time.monotonic() - t0 < secs:
        i += 1
        while len(pending) >= 2: bd.wait_event(pending.popleft(), timeout=20.0)
        R = rotation(time.monotonic() - t0)
        u = [(0.0, 0.0, 0.0, 0.0)] * ub
        u[uv['uScale'][0]] = (size / W, size / H, 0.0, 0.0)
        for k in range(2): u[uv[f'uR{k}'][0]] = R[k] + (0.0,)
        axis = max(range(3), key=lambda k: abs(R[2][k]))         # the stack facing the viewer most; R[2] = the view's z in the volume
        off, count = start[(axis, R[2][axis] > 0.0)]             # +axis towards the viewer: ascending is back to front
        with bd.batch():
            bd.frame_begin()
            bd.vs_uniforms(0, u + vconst)
            bd.draw(0, count, 0, index_handle=H_IBO, index_offset=off)
            bd.flush(H_IMG[i & 1], fp.FL_CLEAR_COLOR | fp.FL_CMASK(0xF), cc, scissor=(0, 0, W, H))
        pending.append(bd.present(H_IMG[i & 1], frame_id=i, hold=False, wait=False))
        now = time.monotonic()
        if now - tm >= 4.0:
            tm = now; s = bd.status()
            print(f"  {now - t0:.0f} s: {(i + 1) / (now - t0):.1f} fps; " + fp.b_frame_split(s))
            sys.stdout.flush()
    while pending: bd.wait_event(pending.popleft(), timeout=20.0)
    el = time.monotonic() - t0
    bd.fence(timeout=30.0)
    for e in bd.take_errors(): print("  EV_ERROR seq %d: %s" % (e[0], err_text(e[:4])))
    print(f"RESULT fb_tex3d_demo: {i + 1} frames in {el:.1f} s = {(i + 1) / el:.1f} fps, the last frame stays on the screen")
    if png:
        write_png(png, bd.mem_read(B_IMG[i & 1], W * H * 4, board=1), W, H, False)
    return 0


if __name__ == '__main__': sys.exit(main())
