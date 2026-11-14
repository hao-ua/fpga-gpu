#!/usr/bin/env python3
"""a_stress_gen — command streams for board A's geometry shaped like a GL 1.x game through host/win/gl (a game of this shape
exposed a lost read in board A's vertex feed: the feed and the assembler busy, the core / setup / binner idle). For
tb_a_stress, which checks progress only (no record golden): every frame must reach its end and the stream its FENCE.

  python3 tools/a_stress_gen.py <seed> [--ndraw N] [--frames F] [--tag T] [--big P] [--lines P] [--reload P] [--nocull]
                                       [--mesh P [--far P] [--meshmin N] [--meshmax N]]
     -> tools/gen/as_<tag><seed>_cmds.hex (ingress-queue bursts, as a_setup_ref.write_cmds), tools/gen/av_<tag><seed>_ddr.hex
     --big: share of large model draws (40-400 vertices); --lines: lines / points; --reload: a VS_PROGRAM after a draw (the
     stage drains); --mesh: indexed model meshes (glDrawElements) whose indices reuse vertices far back (--far: the share)
  Streams that cover the vertex feed's read accounting (hl4 with +MIGJIT_80 stalls without it):
     hl1  --ndraw 600 --frames 2                hl2  --ndraw 800 --frames 2 --big 0.15
     hl4  --ndraw 800 --frames 2 --reload 0.08  hl6  --ndraw 600 --frames 3
     hl21 --ndraw 300 --frames 1 --mesh 0.9 --meshmin 34 --meshmax 90 --far 0.4

What the GL library sends (gl_draw.c emit_one): every draw INDEXED, u16 indices rebased to the draw's lowest vertex, a
triangles / lines / points LIST (to_list: GL_POLYGON -> (i, i+1, 0), GL_QUADS -> two per quad, strips and fans unrolled), the
attribute arrays gathered into the stream one after the other (each its own tight stride), the index list in the stream too;
VERTEX_ATTR for every attribute whose offset moved (all enabled ones, every draw), VS_UNIFORMS every draw, VS_STATE when the
viewport / cull / draw id changes, DRAW. Draw sizes from the game's counts (300-900 draws a frame, ~10 primitives a draw: most
are quads and small polygons, a few models of hundreds)."""
import os, sys, random
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import a_vtx_ref as av, a_setup_ref as asr, isa64
from a_vtx_ref import fb, encode, T_BYTE, T_UBYTE, T_SHORT, T_USHORT, T_FIXED, T_FLOAT, T_FP24, ESZ

GEN = isa64.GEN
FAR = 0.15                                                      # a mesh index reused far back (--far)
T = dict(CREATE_BUFFER=0x10, FENCE=0x40, VS_PROGRAM=0x51, VS_UNIFORMS=0x52, VERTEX_ATTR=0x53, VS_STATE=0x54, DRAW=0x55, FRAME_BEGIN=0x56)
# word addresses in A's host heap (as a_vtx_ref's bases): the program, the index stream, the vertex stream
PROG_W, IDX_W, STR_W = 0x6006000, 0x6040000, 0x6100000
PROG_SZ, IDX_SZ, STR_SZ = 0x4000, 0x100000, 0x800000            # bytes
H_STR, H_IDX, H_PROG = 1, 3, 4


def arg(a, k, d, f=int): return f(a[a.index(k) + 1]) if k in a else d


def to_list(kind, n, rng=None):
    """host/win/gl/gl_draw.c to_list, on vertices 0 .. n-1 -> (DRAW mode, indices). 'mesh': glDrawElements of an indexed
    model (a triangle list over n vertices, mostly local with some vertices reused far back: the vertex cache hits slots
    whose earlier references are all released)"""
    if kind == 'mesh':
        out = []
        for t in range(2 * n):
            c = min(n - 1, t // 2)
            for _ in range(3):
                out.append(rng.randrange(n) if rng.random() < FAR else max(0, min(n - 1, c + rng.randrange(-3, 3))))
        return 0, out
    if kind == 'polygon': return 0, [x for i in range(1, n - 1) for x in (i, i + 1, 0)]
    if kind == 'quads':   return 0, [x for i in range(0, n - 3, 4) for x in (i, i + 1, i + 3, i + 1, i + 2, i + 3)]
    if kind == 'strip':   return 0, [x for i in range(n - 2) for x in ((i + 1, i, i + 2) if i & 1 else (i, i + 1, i + 2))]
    if kind == 'fan':     return 0, [x for i in range(1, n - 1) for x in (0, i, i + 1)]
    if kind == 'tris':    return 0, list(range(n - n % 3))
    if kind == 'lines':   return 4, list(range(n - n % 2))
    if kind == 'lstrip':  return 4, [x for i in range(n - 1) for x in (i, i + 1)]
    return 3, list(range(n))                                    # points


def main():
    a = sys.argv[1:]
    seed = int(a[0]) if a and not a[0].startswith('--') else 1
    ndraw, frames, tag = arg(a, '--ndraw', 600), arg(a, '--frames', 2), arg(a, '--tag', 'hl', str)
    p_big, p_lines, p_reload = arg(a, '--big', 0.04, float), arg(a, '--lines', 0.03, float), arg(a, '--reload', 0.01, float)
    p_mesh = arg(a, '--mesh', 0.0, float)                       # (0 by default: the streams of seeds made before it stay the same)
    global FAR
    FAR = arg(a, '--far', FAR, float)
    mesh_n = (arg(a, '--meshmin', 20), arg(a, '--meshmax', 300))
    rng = random.Random(seed * 7919 + 13)
    memb = {}

    def putb(addr, bs):
        for k, v in enumerate(bs): memb[addr + k] = v
    # ---- the program buffer (tests/av_vs.sasm, the bench program of tb_a_vertex)
    words, _ = isa64.assemble(open(os.path.join(GEN, '..', '..', 'tests', 'av_vs.sasm')).read().splitlines(), vs=True)
    for k, w in enumerate(words):
        putb(4 * (PROG_W + 2 * k), (w & 0xFFFFFFFF).to_bytes(4, 'little')); putb(4 * (PROG_W + 2 * k + 1), (w >> 32).to_bytes(4, 'little'))
    bursts = []; seq = [0]

    def cmd(t, body):
        seq[0] += 1
        bursts.append([(0x51 << 24) | (T[t] << 16) | (4 * len(body)), seq[0], 0, 0, 0, 0, 0, 0])
        for k in range(0, len(body), 8): bursts.append(body[k:k + 8] + [0] * (8 - len(body[k:k + 8])))
    wid, hgt = 640, 480
    vp = [asr.enc(wid * 8), asr.enc(wid * 8), asr.enc(hgt * 8), asr.enc(hgt * 8), asr.enc(asr.F(1, 2)), asr.enc(asr.F(1, 2))]
    gb = asr.guard_consts(vp)
    sp, ip = 0, 0                                               # stream / index stream write pointers (bytes into the buffers)
    # the attribute slots the fixed-function program of the GL library uses: 0 position, 1 colour, 2 / 3 / 4 texture units
    desc = [None] * 16                                          # what the stage holds: (en, handle, size, ty, nm, stride, off)
    nprim = nidx_all = 0; kinds = {}
    for f in range(frames):
        cmd('FRAME_BEGIN', [0])
        if f == 0:
            for h, wbase, size in ((H_STR, STR_W, STR_SZ), (H_IDX, IDX_W, IDX_SZ), (H_PROG, PROG_W, PROG_SZ)):
                cmd('CREATE_BUFFER', [h, 4 * wbase, size])
            cmd('VS_PROGRAM', [H_PROG, 0, len(words)])
            for i in range(16):                                 # every slot once: disabled, its constant
                cmd('VERTEX_ATTR', [i | (0 << 8) | (4 << 16) | (T_FP24 << 20), 0, 0, 0] + [fb(0.25 * (i % 4)), fb(0.5), fb(0.0), fb(1.0)])
                desc[i] = (0, 0, 4, T_FP24, 0, 0, 0)
        did = rng.randrange(1, 4096)
        for k in range(ndraw):
            # ---- the primitive: mostly quads and small polygons, some triangle lists of models, a few lines / points
            r = rng.random()
            if r < p_mesh: kind = 'mesh'
            elif r < p_mesh + p_lines: kind = rng.choice(['lines', 'lines', 'lstrip', 'points'])
            elif r < p_mesh + p_lines + p_big: kind = rng.choice(['tris', 'strip', 'fan'])
            else: kind = rng.choice(['quads', 'quads', 'polygon', 'polygon', 'polygon', 'tris', 'strip', 'fan'])
            if kind == 'mesh': nv = rng.randrange(*mesh_n)
            elif kind in ('tris', 'strip', 'fan') and p_mesh + p_lines <= r < p_mesh + p_lines + p_big: nv = rng.randrange(40, 400)
            elif kind == 'quads': nv = 4 * rng.choice([1, 1, 1, 2, 3, 8, 20])
            elif kind == 'polygon': nv = rng.choice([3, 4, 4, 4, 5, 6, 7, 8, 12, 16, 24])
            elif kind == 'tris': nv = 3 * rng.choice([1, 1, 2, 4, 10])
            else: nv = rng.randrange(2, 30)
            mode, idx = to_list(kind, nv, rng)
            if not idx: continue
            kinds[kind] = kinds.get(kind, 0) + 1
            # ---- the attribute arrays, one after the other in the stream (the library's gather), 4-byte aligned
            lay = {0: (rng.choice([3, 3, 3, 4]), T_FLOAT, 0)}
            cr = rng.random()
            if cr < 0.45: lay[1] = (4, T_UBYTE, 1)
            elif cr < 0.6: lay[1] = (4, T_FLOAT, 0)             # else: a constant colour (glColor before glBegin)
            lay[2] = (2, T_FLOAT, 0)
            if rng.random() < 0.6: lay[3] = (2, T_FLOAT, 0)      # the lightmap's coordinates
            if rng.random() < 0.15: lay[4] = (2, T_FLOAT, 0)     # a detail texture
            for slot in range(5):
                if slot not in lay:
                    if desc[slot][0]:                           # an array turned off: its constant
                        cmd('VERTEX_ATTR', [slot | (4 << 16) | (T_FP24 << 20), 0, 0, 0] + [fb(rng.uniform(0, 1)) for _ in range(3)] + [fb(1.0)])
                        desc[slot] = (0, 0, 4, T_FP24, 0, 0, 0)
                    continue
                sz, ty, nm = lay[slot]
                stride = sz * ESZ[ty]
                if sp + nv * stride + 64 > STR_SZ: sp = 0        # the stream wraps
                off = sp; sp = (sp + nv * stride + 3) & ~3
                for v in range(nv):
                    if slot == 0: vals = [rng.uniform(-2, 2), rng.uniform(-2, 2), rng.uniform(-1, 1), 1.0]
                    elif slot == 1: vals = [rng.uniform(0, 1) for _ in range(4)]
                    else: vals = [rng.uniform(-2, 4) for _ in range(2)]
                    for c in range(sz): putb(4 * STR_W + off + v * stride + c * ESZ[ty], encode(vals[c], ty, nm))
                cmd('VERTEX_ATTR', [slot | (1 << 8) | (sz << 16) | (ty << 20) | (nm << 24), H_STR, stride, off] + [0, 0, 0, fb(1.0)])
                desc[slot] = (1, H_STR, sz, ty, nm, stride, off)
            # ---- the index list (u16, rebased), 2-byte aligned in its stream
            if ip + 2 * len(idx) + 32 > IDX_SZ: ip = 0
            ioff = ip
            for j, x in enumerate(idx): putb(4 * IDX_W + ioff + 2 * j, x.to_bytes(2, 'little'))
            ip += 2 * len(idx)
            # ---- the uniforms (MVP rows and, now and then, more: the texture matrices / colours of the FF program)
            mvp = [[fb(0.45 + rng.uniform(-0.1, 0.1)), fb(rng.uniform(-0.1, 0.1)), fb(rng.uniform(-0.1, 0.1)), fb(rng.uniform(-0.2, 0.2))],
                   [fb(rng.uniform(-0.1, 0.1)), fb(0.45 + rng.uniform(-0.1, 0.1)), fb(rng.uniform(-0.1, 0.1)), fb(rng.uniform(-0.2, 0.2))],
                   [fb(rng.uniform(-0.1, 0.1)), fb(rng.uniform(-0.1, 0.1)), fb(0.4), fb(rng.uniform(-0.2, 0.2))],
                   [fb(rng.uniform(-0.15, 0.15)), fb(rng.uniform(-0.15, 0.15)), fb(rng.uniform(-0.45, 0.3)), fb(1.0)]]
            nu = rng.choice([4, 4, 4, 8, 12, 24])
            rows = mvp + [[fb(rng.uniform(-1, 1)) for _ in range(4)] for _ in range(nu - 4)]
            cmd('VS_UNIFORMS', [(nu << 16) | 0] + [c for row in rows for c in row])
            # ---- the draw's state when it changes (the library: viewport, cull, draw id; HL changes cull and the id often)
            if k == 0 or rng.random() < 0.35:
                did = (did + 1) & 0x1FFF or 1
                cull = 0 if '--nocull' in a else rng.choice([0, 0, 1, 2])
                fsmask = rng.choice([0x27, 0x27, 0x03, 0x01, 0x07])
                lw = rng.choice([1, 1, 2])
                cmd('VS_STATE', vp[:4] + [cull | (rng.randrange(2) << 8) | (lw << 9) | (fsmask << 16), wid | (hgt << 16), did] + vp[4:] + gb)
            cmd('DRAW', [mode | (1 << 8), H_IDX, ioff, 0, len(idx)])
            nprim += len(idx) // (3 if mode == 0 else 2 if mode == 4 else 1); nidx_all += len(idx)
            if rng.random() < p_reload: cmd('VS_PROGRAM', [H_PROG, 0, len(words)])   # a program change (the stage drains)
    cmd('FENCE', [0x5EED])
    name = f"{tag}{seed}"
    with open(f"{GEN}/as_{name}_cmds.hex", 'w') as fh:
        for bw in bursts: fh.write(''.join(f"{bw[7 - k]:08x}" for k in range(8)) + "\n")
    bs = {}
    for ad, v in memb.items(): bs[ad >> 5] = bs.get(ad >> 5, 0) | (v << (8 * (ad & 31)))
    with open(f"{GEN}/av_{name}_ddr.hex", 'w') as fh:
        for b in sorted(bs): fh.write(f"{b:08x} {bs[b]:064x}\n")
    print(f"as_{name}: {frames} frame(s) x {ndraw} draws, {nprim} primitives, {nidx_all} indices, {len(bursts)} queue bursts, {len(bs)} DDR bursts; "
          f"kinds {dict(sorted(kinds.items()))}")


if __name__ == '__main__': main()
