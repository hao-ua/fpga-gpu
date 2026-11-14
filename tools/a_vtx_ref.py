#!/usr/bin/env python3
"""a_vtx_ref — model of board A's vertex front end (src/rtl/a_vertex.sv): a random draw — attribute
buffers and an index buffer as a DDR image, the 16 attribute descriptors, a vertex program — the vertex program run on
every referenced vertex with isa64.simulate_vs2 (the same core model tb_vs2 uses), and the stream of primitive vertices
the front end must present to the setup stage, one line per vertex: gl_Position then varyings 0..7 (unread ones 0).

  python3 tools/a_vtx_ref.py <seed> [--mode tri|strip|fan] [--nv N] [--nidx N] [--arrays] [--u8] [--first N] [--ioff N] [--types] [--far]
     (--far: some vertices far outside the view in x / y, beyond the +-2048 px guard band: the x / y clip planes)
     (--ioff N: the indices start N elements into the index buffer — an unaligned DrawElements offset)
     (--types: the four read attributes in client formats chosen by the seed — BYTE / UNSIGNED_BYTE / SHORT /
      UNSIGNED_SHORT / FIXED / FLOAT / FP24, normalized or not, at byte offsets and strides with random padding, ES 2.0 §2.8;
      without it the layout is the original one: FP24 words, VBO A {pos4, col4, uv2, pad2} stride 48 bytes, VBO B nrm3)
     -> tools/gen/av_<seed>_ddr.hex      "burst-index data256" per written burst (byte address >> 5; byte b at [8b +: 8])
        tools/gen/av_<seed>_draw.mem     {varmask[7:0], idx_base[29:0] (bytes), count[31:0], first[31:0], u8, indexed, mode[2:0]}
        tools/gen/av_<seed>_attr.mem     16 lines {const[95:0], off[29:0], stride[29:0], norm, type[2:0], size[2:0], en, base[29:0]} (bytes)
        tools/gen/av_<seed>_iram.mem, _params.mem (the program tests/av_vs.sasm, 256 uniforms)
        tools/gen/av_<seed>_expect.mem   one vertex per line: vec4hex(position) + 8 x vec4hex(varying) + vec4hex(gl_PointSize row)
        tools/gen/av_<seed>_info.txt     counts (vertices, indices, primitives, unique vertices)
Buffers are BYTE addressed (the DDR image is sparse; a burst = 32 bytes); d['bases'] are 32-byte-aligned WORD addresses for
CREATE_BUFFER (bytes = 4 x). Indices are u16 or u8 (--u8), little endian. The conversion of a component to FP24 is
conv() below — the hardware's attr_conv, bit for bit: normalized signed (2c + 1) / (2^b - 1), unsigned c / (2^b - 1),
FIXED c / 2^16, integers exactly, FLOAT with round-to-nearest-even and flush-to-zero (float_ref.encode_real). Components
beyond an attribute's size read (0, 0, 0, 1); a disabled attribute supplies its constant."""
import os, sys, random, struct
from fractions import Fraction as F
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import float_ref as fr, isa64

FMT = fr.FP24; GEN = isa64.GEN
ONE = fr.encode_real(F(1), FMT)
MODES = {'tri': 0, 'strip': 1, 'fan': 2, 'points': 3, 'lines': 4, 'lstrip': 5, 'lloop': 6}   # the DRAW mode field
# VERTEX_ATTR type codes (attr_conv.sv)
T_BYTE, T_UBYTE, T_SHORT, T_USHORT, T_FIXED, T_FLOAT, T_FP24 = range(7)
TNAME = {T_BYTE: 'byte', T_UBYTE: 'ubyte', T_SHORT: 'short', T_USHORT: 'ushort', T_FIXED: 'fixed', T_FLOAT: 'float', T_FP24: 'fp24'}
ESZ = {T_BYTE: 1, T_UBYTE: 1, T_SHORT: 2, T_USHORT: 2, T_FIXED: 4, T_FLOAT: 4, T_FP24: 4}


def fb(x): return fr.encode_real(F(x).limit_denominator(1 << 16), FMT)


# ---------------------------------------------------------------- the component formats (attr_conv.sv)
def conv(raw, ty, nm):
    """one component's bytes (little endian, ESZ[ty] of them) -> FP24 bits, as the hardware converts it"""
    if ty == T_FP24:
        return int.from_bytes(raw, 'little') & 0xFFFFFF
    if ty == T_FLOAT:
        w = int.from_bytes(raw, 'little'); s = w >> 31; e = (w >> 23) & 0xFF; m = w & 0x7FFFFF
        if e == 0xFF: return (s << 23) | (0x7F << 16) | (0 if m == 0 else 0x8000)     # Inf / a quiet NaN with the sign
        if e == 0: return s << 23                                                      # zero / denormal: FTZ, the sign kept
        return fr.encode_real(F(struct.unpack('<f', raw)[0]), FMT)
    if ty == T_FIXED:
        return fr.encode_real(F(int.from_bytes(raw, 'little', signed=True), 1 << 16), FMT)
    signed = ty in (T_BYTE, T_SHORT); b = 8 * ESZ[ty]
    c = int.from_bytes(raw, 'little', signed=signed)
    if nm: v = F(2 * c + 1, (1 << b) - 1) if signed else F(c, (1 << b) - 1)
    else:  v = F(c)
    return fr.encode_real(v, FMT)


def encode(x, ty, nm):
    """a real value -> the component's bytes in format (ty, nm): the nearest representable value (clamped)"""
    def clamp(v, lo, hi): return max(lo, min(hi, v))
    if ty == T_FP24:  return (fb(x) & 0xFFFFFF).to_bytes(4, 'little')
    if ty == T_FLOAT: return struct.pack('<f', float(x))
    if ty == T_FIXED: return clamp(round(x * 65536), -(1 << 31), (1 << 31) - 1).to_bytes(4, 'little', signed=True)
    signed = ty in (T_BYTE, T_SHORT); b = 8 * ESZ[ty]
    if nm: c = round((x * ((1 << b) - 1) - 1) / 2) if signed else round(x * ((1 << b) - 1))
    else:  c = round(x)
    c = clamp(c, -(1 << (b - 1)) if signed else 0, (1 << (b - 1)) - 1 if signed else (1 << b) - 1)
    return c.to_bytes(ESZ[ty], 'little', signed=signed)


def primitives(mode, n):
    """ES 2.0 §2.6.1: TRIANGLES, TRIANGLE_STRIP (odd triangles with the first two vertices swapped), TRIANGLE_FAN;
    POINTS (3), LINES (4), LINE_STRIP (5), LINE_LOOP (6: the strip plus the closing segment last -> first)."""
    if mode == 0: return [(3 * i, 3 * i + 1, 3 * i + 2) for i in range(n // 3)]
    if mode == 1: return [((i, i + 1, i + 2) if i % 2 == 0 else (i + 1, i, i + 2)) for i in range(n - 2)]
    if mode == 2: return [(0, i + 1, i + 2) for i in range(n - 2)]
    if mode == 3: return [(i,) for i in range(n)]
    if mode == 4: return [(2 * i, 2 * i + 1) for i in range(n // 2)]
    if mode == 5: return [(i, i + 1) for i in range(n - 1)]
    return [(i, i + 1) for i in range(n - 1)] + ([(n - 1, 0)] if n >= 2 else [])


def prim_size(mode): return 3 if mode <= 2 else 1 if mode == 3 else 2


def parse_args(a):
    seed = int(a[0]) if a and not a[0].startswith('--') else 1
    mode = MODES[a[a.index('--mode') + 1]] if '--mode' in a else 0
    nv = int(a[a.index('--nv') + 1]) if '--nv' in a else 40
    nidx = int(a[a.index('--nidx') + 1]) if '--nidx' in a else 90
    first = int(a[a.index('--first') + 1]) if '--first' in a else 0
    ioff = int(a[a.index('--ioff') + 1]) if '--ioff' in a else 0      # the index buffer's offset in indices (byte offset = ioff x 1 or 2)
    return seed, mode, nv, nidx, '--arrays' not in a, '--u8' in a, first, ioff, '--types' in a, 'huge' if '--huge' in a else '--far' in a


# the typed layouts of --types: per read attribute (0 position, 1 colour, 2 uv, 3 normal) the formats a seed may pick:
# (type, normalized, components); the value ranges stay those of the FP24 layout so the draws look alike
LAYOUTS = {0: [(T_FLOAT, 0, 4), (T_FIXED, 0, 4), (T_FP24, 0, 4), (T_SHORT, 1, 3), (T_FLOAT, 0, 3)],
           1: [(T_UBYTE, 1, 4), (T_USHORT, 1, 4), (T_FLOAT, 0, 4), (T_UBYTE, 0, 4), (T_SHORT, 0, 4), (T_BYTE, 1, 4), (T_FIXED, 0, 3)],
           2: [(T_FLOAT, 0, 2), (T_FIXED, 0, 2), (T_USHORT, 0, 2), (T_SHORT, 1, 2), (T_BYTE, 0, 2), (T_FP24, 0, 2)],
           3: [(T_BYTE, 1, 3), (T_SHORT, 1, 3), (T_FLOAT, 0, 3), (T_FIXED, 0, 3), (T_FP24, 0, 3), (T_USHORT, 1, 3)]}


def run_vs(words, attrs, params, uniq, varmask):
    """the vertex program on the unique vertices (16 per warp, as the core runs them) with these uniforms ->
    vertex id -> (position, varyings[8] (unread ones 0), the gl_PointSize row)"""
    res = {}
    for b in range(0, len(uniq), 16):
        grp = uniq[b:b + 16]
        varin = [attrs[v] for v in grp] + [attrs[grp[0]]] * (16 - len(grp))
        r = isa64.simulate_vs2(words, isa64.FSInputs(params, varin, interp=False))
        assert not r.err
        for l, v in enumerate(grp):
            res[v] = (r.outc[l], [r.outv[i][l] if (varmask >> i) & 1 else [0] * 4 for i in range(8)], r.psize[l])
    return res


def build_draw(seed, mode=0, nv=40, nidx=90, indexed=True, u8=False, first=0, ioff=0, types=False, write=True, proj=False, far=False):
    """The random draw: returns a dict (memb: byte address -> byte, mem: word address -> word, desc (16 x (en, base,
    size, type, normalized, stride, offset) in bytes), consts, idx, words, params, varmask, prims, res: vertex id ->
    (position, varyings[8], point size), attrs, bases (word addresses of the four buffers)) and writes the av_<seed>
    files when write."""
    rng = random.Random(seed)
    memb = {}                                             # byte address -> byte

    def putb(addr, bs):
        for k, v in enumerate(bs): memb[addr + k] = v
    # word addresses in A's host heap (384 MB..): VBO A, VBO B, the index buffer, the program buffer
    base_a, base_b, idx_base, prog_base = 0x6002000, 0x6004000, 0x6001000, 0x6006000
    ba, bb = 4 * base_a, 4 * base_b                       # bytes
    # ---- the layouts: (type, normalized, size) and the byte placement of the four read attributes
    if types:
        fmts = [rng.choice(LAYOUTS[k]) for k in range(4)]
        in_b = rng.choice([3, 3, 1, 2])                    # which attribute lives alone in VBO B (the others interleave in A)
        offs = {}; off = 0
        for k in range(4):
            if k == in_b: continue
            off += rng.choice([0, 0, 1, 2, 3])            # a random pad: any byte alignment
            offs[k] = off; off += fmts[k][2] * ESZ[fmts[k][0]]
        stride_a = off + rng.choice([0, 0, 1, 4])
        stride_b = fmts[in_b][2] * ESZ[fmts[in_b][0]] + rng.choice([0, 0, 1, 2])
        off_b = rng.choice([0, 0, 1, 3])
        desc = []
        for k in range(4):
            ty, nm, sz = fmts[k]
            desc.append((1, bb, sz, ty, nm, stride_b, off_b) if k == in_b else (1, ba, sz, ty, nm, stride_a, offs[k]))
    else:                                                 # the original layout: FP24 words, VBO A {pos4, col4, uv2, pad2}, VBO B nrm3
        fmts = [(T_FP24, 0, 4), (T_FP24, 0, 4), (T_FP24, 0, 2), (T_FP24, 0, 3)]
        desc = [(1, ba, 4, T_FP24, 0, 48, 0), (1, ba, 4, T_FP24, 0, 48, 16), (1, ba, 2, T_FP24, 0, 48, 32), (1, bb, 3, T_FP24, 0, 12, 0)]
    desc += [(0, 0, 4, T_FP24, 0, 0, 0)] * 12
    # ---- the vertices: ids 0 .. nv - 1 (indexed) or first .. first + nidx - 1 (arrays); values in the layout's ranges
    #      (the rng call order of the original layout is kept: the goldens of seeds 1..20 do not move)
    vids = list(range(nv)) if indexed else list(range(first, first + nidx))
    attrs = {}; vecs = {}                                 # vertex id -> 16 vec4 (FP24 bits, as the core receives them)
    for v in vids:
        pos_r = [rng.uniform(-2, 2), rng.uniform(-2, 2), rng.uniform(-1, 1), rng.choice([1, 1, 2, 0.5])]
        if fmts[0][1]: pos_r = [rng.uniform(-1, 1), rng.uniform(-1, 1), rng.uniform(-1, 1), 1.0]          # normalized: |v| <= 1
        if far and mode == 3:                                        # --far points: centres around the top of the view (ndc y ~ 1 with
            pos_r = [rng.uniform(-0.5, 0.5), rng.uniform(2.0, 2.5), rng.uniform(-1, 1), 1.0]   # the proj MVP: the +-2048 px limit of a tall target)
        elif far == 'huge':                                          # --huge: every vertex x / y 60 x, so most triangles span the guard
            if not fmts[0][1]: pos_r = [pos_r[0] * 60, pos_r[1] * 60, pos_r[2], pos_r[3]]   # band: |a2| >= 2^30, the edge shift s > 0
        elif far:                                                    # --far: some vertices well outside the guard band (x / y clipping)
            fct = rng.choice([1.0, 1.0, 3.0, 8.0, 25.0])
            if fct != 1.0 and not fmts[0][1]: pos_r = [pos_r[0] * fct, pos_r[1] * fct, pos_r[2], pos_r[3]]
        col_r = [rng.uniform(0, 1) for _ in range(4)]
        if fmts[1][0] in (T_UBYTE, T_SHORT) and not fmts[1][1]: col_r = [rng.uniform(0, 200) for _ in range(4)]   # integers
        if fmts[1][0] == T_BYTE and fmts[1][1]: col_r = [rng.uniform(-1, 1) for _ in range(4)]
        uv_r = [rng.uniform(0, 4) for _ in range(2)]
        if fmts[2][1]: uv_r = [rng.uniform(-1, 1) for _ in range(2)]
        if fmts[2][0] == T_BYTE: uv_r = [rng.uniform(-4, 4) for _ in range(2)]
        nrm_r = [rng.uniform(-1, 1) for _ in range(3)]
        if fmts[3][0] == T_USHORT and fmts[3][1]: nrm_r = [rng.uniform(0, 1) for _ in range(3)]
        vals = [pos_r, col_r, uv_r, nrm_r]
        vec = []
        for k in range(4):
            en, base, sz, ty, nm, stride, off = desc[k]
            comps = []
            for c in range(sz):
                raw = encode(vals[k][c], ty, nm)
                putb(base + v * stride + off + c * ESZ[ty], raw)
                comps.append(conv(raw, ty, nm))
            vec.append(comps + [0, 0, 0, ONE][sz:])        # (0, 0, 0, 1) beyond the size
        vecs[v] = vec
    consts = [[fb(rng.uniform(-3, 3)) for _ in range(4)] for _ in range(16)]
    for v in vids: attrs[v] = vecs[v] + [consts[i] for i in range(4, 16)]
    # ---- indices (with reuse, so the vertex cache hits) or arrays first..first+count
    if indexed:
        idx = [rng.randrange(0, min(nv, 256) if u8 else nv) for _ in range(nidx)]
        if mode != 0:                                      # strips / fans: avoid degenerate repeats of the same vertex
            for i in range(1, nidx):
                while idx[i] == idx[i - 1]: idx[i] = rng.randrange(0, min(nv, 256) if u8 else nv)
        # index k lives at element position ioff + k (DrawElements' indices pointer need not be word aligned)
        for k, ix in enumerate(idx):
            e = ioff + k
            putb(4 * idx_base + e * (1 if u8 else 2), ix.to_bytes(1 if u8 else 2, 'little'))
    else:
        idx = [first + i for i in range(nidx)]
    # ---- the program and uniforms
    words, _ = isa64.assemble(open(os.path.join(GEN, '..', '..', 'tests', 'av_vs.sasm')).read().splitlines(), vs=True)
    if proj:                                              # a projection-like MVP: most vertices in front of the eye and on screen
        params = [[fb(0.45 + rng.uniform(-0.1, 0.1)), fb(rng.uniform(-0.1, 0.1)), fb(rng.uniform(-0.1, 0.1)), fb(rng.uniform(-0.2, 0.2))],
                  [fb(rng.uniform(-0.1, 0.1)), fb(0.45 + rng.uniform(-0.1, 0.1)), fb(rng.uniform(-0.1, 0.1)), fb(rng.uniform(-0.2, 0.2))],
                  [fb(rng.uniform(-0.1, 0.1)), fb(rng.uniform(-0.1, 0.1)), fb(0.4), fb(rng.uniform(-0.2, 0.2))],
                  [fb(rng.uniform(-0.15, 0.15)), fb(rng.uniform(-0.15, 0.15)), fb(rng.uniform(-0.45, 0.3)), fb(1.0)]]   # some w < 2^-7: clipping
        params += [[fb((i % 7) * 0.25 + 0.5)] * 4 for i in range(252)]
        if far == 'huge':                                 # z and w without x / y terms: the 60 x positions reach NDC +-50
            params[2][0] = params[2][1] = params[3][0] = params[3][1] = fb(0.0)   # and the near / far planes leave them wide
    else:
        params = [[fb(rng.uniform(-1.5, 1.5)) for _ in range(4)] for _ in range(4)] + [[fb((i % 7) * 0.25 + 0.5)] * 4 for i in range(252)]
    varmask = 0x27                                        # the varyings av_vs.sasm writes: 0, 1, 2, 5
    for k, w in enumerate(words):                         # the program as a buffer: 32-bit words, the low half first
        putb(4 * (prog_base + 2 * k), (w & 0xFFFFFFFF).to_bytes(4, 'little')); putb(4 * (prog_base + 2 * k + 1), (w >> 32).to_bytes(4, 'little'))
    # ---- the vertex program on every unique vertex (16 per warp, as the core runs them)
    uniq = sorted(set(idx))
    res = run_vs(words, attrs, params, uniq, varmask)
    prims = primitives(mode, nidx)
    mem = {}                                              # the word view (fb_geom_test's uploads)
    for a, v in memb.items(): mem[a >> 2] = mem.get(a >> 2, 0) | (v << (8 * (a & 3)))
    d = dict(seed=seed, mode=mode, indexed=indexed, u8=u8, first=first, ioff_bytes=ioff * (1 if u8 else 2), nv=nv, nidx=nidx, memb=memb, mem=mem,
             desc=desc, consts=consts, fmts=fmts, idx=idx, words=words, params=params, varmask=varmask, prims=prims, res=res, attrs=attrs,
             idx_base=idx_base, uniq=uniq, bases=dict(a=base_a, b=base_b, idx=idx_base, prog=prog_base))
    if write: write_av(d)
    return d


def write_av(d):
    memb, desc, consts, idx, words, params, varmask, prims, res = (d[k] for k in ('memb', 'desc', 'consts', 'idx', 'words', 'params', 'varmask', 'prims', 'res'))
    seed, mode, indexed, u8, first, nidx, idx_base, uniq, nv, fmts = (d[k] for k in ('seed', 'mode', 'indexed', 'u8', 'first', 'nidx', 'idx_base', 'uniq', 'nv', 'fmts'))
    tag = f"av_{seed}"
    bursts = {}
    for a, v in memb.items(): bursts[a >> 5] = bursts.get(a >> 5, 0) | (v << (8 * (a & 31)))
    with open(f"{GEN}/{tag}_ddr.hex", 'w') as f:
        for b in sorted(bursts): f.write(f"{b:08x} {bursts[b]:064x}\n")
    with open(f"{GEN}/{tag}_draw.mem", 'w') as f:            # 107 bits: {varmask, idx_base (bytes), count, first, u8, indexed, mode}
        v = (varmask << 99) | ((4 * idx_base + d['ioff_bytes']) << 69) | (nidx << 37) | (first << 5) | (int(u8) << 4) | (int(indexed) << 3) | mode
        f.write(f"{v:027x}\n")
    with open(f"{GEN}/{tag}_attr.mem", 'w') as f:            # 194 bits: {const, off, stride, norm, type, size, en, base}
        for i, (en, base, size, ty, nm, stride, off) in enumerate(desc):
            c = consts[i]; cv = (c[3] << 72) | (c[2] << 48) | (c[1] << 24) | c[0]
            v = (cv << 98) | (off << 68) | (stride << 38) | (nm << 37) | (ty << 34) | (size << 31) | (en << 30) | base
            f.write(f"{v:049x}\n")
    isa64.write_words(words, f"{GEN}/{tag}_iram.mem")
    with open(f"{GEN}/{tag}_params.mem", 'w') as f:
        for v in params: f.write(isa64.vec4hex(v) + "\n")
    with open(f"{GEN}/{tag}_expect.mem", 'w') as f:            # per primitive vertex: position, 8 varyings, the gl_PointSize row
        for p in prims:
            for k in p:
                pos, var, ps = res[idx[k]]
                f.write(isa64.vec4hex(pos) + ''.join(isa64.vec4hex(var[i]) for i in range(8)) + isa64.vec4hex(ps) + "\n")
    layout = ' '.join(f"{TNAME[ty]}{'n' if nm else ''}{sz}@{off}/{stride}" for (en, base, sz, ty, nm, stride, off) in desc[:4])
    with open(f"{GEN}/{tag}_info.txt", 'w') as f:
        f.write(f"mode {mode} indexed {int(indexed)} u8 {int(u8)} first {first} vertices {nv} indices {nidx} unique {len(uniq)} primitives {len(prims)} layout {layout}\n")
    print(f"{tag}: mode {mode} {'indexed' if indexed else 'arrays'}{' u8' if u8 else ''}: {nidx} indices, {len(uniq)} unique "
          f"vertices, {len(prims)} primitives, attributes {layout} -> tools/gen/{tag}_*.mem/.hex")


def main():
    *pos, far = parse_args(sys.argv[1:])                  # far is the last value; positionally it would land in `write`
    build_draw(*pos, far=far)


if __name__ == '__main__': main()
