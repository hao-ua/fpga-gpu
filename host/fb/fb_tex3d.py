#!/usr/bin/env python3
"""fb_tex3d — 3D textures on the two boards (tmu24's volume path, on a board B built with 3D textures): volumes
created and uploaded slice by slice, a full-screen quad sampling them, the frame read back from B's DDR and pixels
compared with tools/tmu_ref.py, the bit-exact model of the TMU, on the coordinates the shader should have computed.

  python3 host/fb/fb_tex3d.py [--ip 192.168.1.10] [--mode M] [--scene NAME] [--list] [--png FILE]
  python3 host/fb/fb_tex3d.py --perf [--secs S]          # cycles per quad at 800 x 600 (RESULT lines)

Scenes: slices one at a time, the blend between two slices, the wrap modes along r, NPOT and depth > width, RGBA8 / RGB565 /
RGBA4444, the LOD from s, t and from r alone, the depth halving with the level, trilinear. The textures are a gradient
(R = x, G = y, B = z, alpha = the level) and a hash (any wrong address shows). The check tolerates what the shader's own
FP24 coordinates add: 3 steps of 255, and for NEAREST the coordinates' last bits may pick the next texel. One scene
compares against a wrong wrap mode and must fail (the check's proof that it can see one); two commands that the command
processor must refuse end the run. The shader is tests/fs_tex3d.frag.
Standard library + the repository's tools.
"""
import array, collections, os, random, struct, sys, time, zlib
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(HERE, '..', '..', 'tools'))
import fbproto as fp
import float_ref as fr
import tmu_ref as tr
from fb_image import MODE_SIZE
from fb_shader import (words64_bytes, f24, H_VBO, H_IBO, H_VSPROG, A_VBO, A_IBO, A_VSPROG, H_FSPROG, B_FSPROG,
                       H_IMG, B_IMG, DRAW_ID, err_text)
from fb_texperf import compile_programs, fs_uniforms, render_frame, B_TEX

NEAREST, BILINEAR, MIP_NEAREST, TRILINEAR, NEAREST_MIPN = 0x0, 0x9, 0xB, 0xD, 0x2
REPEAT, CLAMP, MIRROR = 0, 1, 2
RGBA8, RGB565, RGBA4444 = fp.FMT_RGBA8, fp.FMT_RGB565, 0x05
TMU_FMT = {RGBA8: 0, RGBA4444: 1, RGB565: 3}
H_VOL = 0xC050                                                  # the volumes' image handles from here


# ------------------------------------------------------------------------------------------------ volumes
class Mem:
    """tmu_ref's word memory as an array"""
    def __init__(self, words): self.a = array.array('I', bytes(4 * words))
    def get(self, w, dflt=0): return self.a[w] if 0 <= w < len(self.a) else dflt


def grad_slice(w, h, d, z, l):
    """R = x, G = y, B = z, alpha = the level"""
    n = w * h
    buf = bytearray(n * 4)
    buf[0::4] = bytes(x * 255 // max(1, w - 1) for x in range(w)) * h
    buf[1::4] = b''.join(bytes((y * 255 // max(1, h - 1),)) * w for y in range(h))
    buf[2::4] = bytes((z * 255 // max(1, d - 1),)) * n
    buf[3::4] = bytes((255 - 32 * l,)) * n
    return buf


def hash_slice(w, h, d, z, l):
    buf = bytearray()
    for y in range(h):
        for x in range(w):
            v = (x * 73856093 ^ y * 19349663 ^ z * 83492791 ^ l * 2654435761) & 0xFFFFFFFF
            v = ((v ^ (v >> 13)) * 1274126177) & 0xFFFFFFFF
            buf += struct.pack("<I", v ^ (v >> 16))
    return buf


class Volume:
    def __init__(self, name, w, h, dep, fmt, levels, slice_fn):
        self.name, self.w, self.h, self.dep, self.fmt, self.levels, self.slice_fn = name, w, h, dep, fmt, levels, slice_fn
        self.d = dict(base=0, w=w, h=h, d=dep, fmt=TMU_FMT[fmt], levels=levels, filt=0, wrap_s=0, wrap_t=0, wrap_r=0,
                      cube=0, vol=1, incomplete=0)
        self.offs, self.chain = tr.level_offsets(self.d)
        self.mem = Mem(self.chain + 16)
        self.handle = 0

    def slices(self):
        """every slice of every level: (level, z, w, h, the bytes to upload), filling the model's memory on the way"""
        d = self.d; a = self.mem.a
        for l in range(self.levels):
            lw, lh = tr.level_dims(d, l); ld = tr.level_depth(d, l)
            for z in range(ld):
                buf = self.slice_fn(lw, lh, ld, z, l)
                T0 = z * lh * lw
                if self.fmt == RGBA8:
                    w = array.array('I'); w.frombytes(bytes(buf)); w.byteswap()   # R in the top byte (FMT_CH)
                    a[self.offs[l] + T0:self.offs[l] + T0 + len(w)] = w
                    yield l, z, lw, lh, bytes(buf)
                else:
                    out = bytearray()
                    for i in range(lw * lh):
                        r, g, b, al = buf[i * 4:i * 4 + 4]
                        v = ((r >> 3) << 11 | (g >> 2) << 5 | (b >> 3)) if self.fmt == RGB565 else \
                            ((r >> 4) << 12 | (g >> 4) << 8 | (b >> 4) << 4 | (al >> 4))
                        T = T0 + i
                        a[self.offs[l] + (T >> 1)] |= v << (16 if T & 1 else 0)
                        out += struct.pack("<H", v)
                    yield l, z, lw, lh, bytes(out)

    def upload(self, bd, handle, base):
        self.handle = handle
        bd.create_image(handle, base, self.w, self.h, self.fmt, levels=self.levels, layers=self.dep, vol=True,
                        pitch=self.w * (4 if self.fmt == RGBA8 else 2), usage=fp.USE_SAMPLED | fp.USE_TRANSFER)
        n = 0
        for l, z, lw, lh, data in self.slices():
            tf = fp.TF_RGBA8888 if self.fmt == RGBA8 else fp.TF_RGB565 if self.fmt == RGB565 else fp.TF_RAW
            bd.write_image(handle, data, tf, 0, 0, lw, lh, level=l, layer=z, storage_fmt=RGBA8 if self.fmt == RGBA8 else RGB565)
            n += len(data)
        return n


VOLS = collections.OrderedDict((v.name, v) for v in (
    Volume('g64',  64, 64, 16, RGBA8, 7, grad_slice),            # POT, a full chain to 1 x 1 x 1
    Volume('g33',  33, 17, 5, RGB565, 6, grad_slice),            # NPOT, the depth runs out first
    Volume('h33',  33, 17, 5, RGB565, 1, hash_slice),
    Volume('h8',   8, 8, 8, RGBA8, 4, hash_slice),
    Volume('h20',  20, 6, 31, RGBA4444, 5, hash_slice),          # depth > width > height, 16 bits, odd levels
    Volume('g128', 128, 128, 128, RGBA8, 8, grad_slice),         # 128^3 with its whole chain (8 levels)
))


# ------------------------------------------------------------------------------------------------ the scenes
Scene = collections.namedtuple('Scene', 'name vol filt ws wt wr off mul')


def scenes():
    out = []
    S = lambda *a: out.append(Scene(*a))
    for k in (0, 1, 7, 15):
        S(f'slice_{k}', 'g64', NEAREST, CLAMP, CLAMP, CLAMP, (0.5, 0.5, (k + 0.5) / 16), (0.45, 0.45, 0.0, 0.0))
    S('between_7_8', 'g64', BILINEAR, CLAMP, CLAMP, CLAMP, (0.5, 0.5, 8 / 16), (0.45, 0.45, 0.0, 0.0))
    S('slice_3q', 'g64', BILINEAR, CLAMP, CLAMP, CLAMP, (0.5, 0.5, 3.75 / 16), (0.45, 0.45, 0.0, 0.0))
    for name, wr in (('repeat', REPEAT), ('clamp', CLAMP), ('mirror', MIRROR)):
        S(f'r_{name}', 'g64', BILINEAR, REPEAT, REPEAT, wr, (0.5, 0.5, 0.5), (0.7, 0.7, 1.75, 0.0))
        S(f'r_near_{name}', 'g64', NEAREST, REPEAT, REPEAT, wr, (0.5, 0.5, 0.5), (0.7, 0.7, 1.0, 1.75))
    S('st_repeat', 'g64', BILINEAR, REPEAT, MIRROR, CLAMP, (0.5, 0.5, 0.4), (2.2, 1.7, 0.1, 0.0))
    # 640 x 480: texels per pixel = 2 * s per x * 64 / 640 = s per x / 5; t per y / 3.75; r: r per x / 20 (16 slices)
    S('lod_1', 'g64', TRILINEAR, REPEAT, REPEAT, CLAMP, (0.5, 0.5, 0.5), (10.0, 7.5, 0.0, 0.0))
    S('lod_1_5', 'g64', TRILINEAR, REPEAT, REPEAT, CLAMP, (0.5, 0.5, 0.5), (14.14, 10.6, 0.0, 0.0))
    S('lod_2_mipn', 'g64', MIP_NEAREST, REPEAT, REPEAT, CLAMP, (0.5, 0.5, 0.5), (20.0, 15.0, 0.0, 0.0))
    S('lod_r_2', 'g64', TRILINEAR, CLAMP, CLAMP, REPEAT, (0.5, 0.5, 0.5), (0.3, 0.3, 80.0, 0.0))    # r alone asks for level 2
    S('lod_r_1_5', 'g64', TRILINEAR, CLAMP, CLAMP, REPEAT, (0.5, 0.5, 0.5), (0.3, 0.3, 56.6, 0.0))
    S('lod_top', 'g64', TRILINEAR, REPEAT, REPEAT, REPEAT, (0.5, 0.5, 0.5), (640.0, 480.0, 0.0, 0.0))
    S('npot_near', 'h33', NEAREST, REPEAT, CLAMP, MIRROR, (0.5, 0.5, 0.5), (0.45, 0.45, 0.45, 0.0))
    S('npot_lin', 'h33', BILINEAR, MIRROR, REPEAT, CLAMP, (0.5, 0.5, 0.5), (0.45, 0.45, 0.45, 0.0))
    S('npot_mip', 'g33', TRILINEAR, REPEAT, REPEAT, CLAMP, (0.5, 0.5, 0.5), (19.4, 14.1, 0.0, 0.0))
    S('npot_mip_lv2', 'g33', NEAREST_MIPN, REPEAT, REPEAT, REPEAT, (0.5, 0.5, 0.5), (38.8, 28.2, 0.0, 0.0))
    S('h8_lin', 'h8', BILINEAR, REPEAT, REPEAT, REPEAT, (0.5, 0.5, 0.5), (0.6, 0.6, 0.6, 0.6))
    S('deep_near', 'h20', NEAREST, CLAMP, CLAMP, CLAMP, (0.5, 0.5, 0.5), (0.45, 0.45, 0.45, 0.0))
    S('deep_lin', 'h20', BILINEAR, CLAMP, CLAMP, MIRROR, (0.5, 0.5, 0.5), (0.45, 0.45, 0.45, 0.2))
    S('g128_lin', 'g128', BILINEAR, REPEAT, REPEAT, CLAMP, (0.5, 0.5, 0.5), (0.45, 0.45, 0.4, 0.0))
    S('g128_tri', 'g128', TRILINEAR, REPEAT, REPEAT, CLAMP, (0.5, 0.5, 0.5), (7.0, 5.3, 3.0, 0.0))
    return out


def sampler_word(sc, handle):
    return handle | (sc.filt << 16) | (sc.ws << 20) | (sc.wt << 22) | (sc.wr << 25)


def set_scene(bd, sc, wf, fconst, funi, fs_cb, unit):
    uni = fs_uniforms(funi, fconst, fs_cb, {'uOff': tuple(sc.off) + (0.0,), 'uMul': tuple(sc.mul)})
    h = [0] * 23
    h[0] = 0xF << 16
    h[11] = len(wf) << 16 | H_FSPROG; h[12] = 0; h[13] = len(uni)
    h[15 + unit] = sampler_word(sc, VOLS[sc.vol].handle)
    bd.draw_state(DRAW_ID, h, uni)


# ------------------------------------------------------------------------------------------------ the check
def model_quad(sc, f, qx, qy, W, H, nudge=0.0):
    """the 4 pixels of the quad at (qx, qy) from tmu_ref: [(R, G, B, A) x 4] in 0..255"""
    v = VOLS[sc.vol]
    d = dict(v.d, filt=sc.filt, wrap_s=sc.ws, wrap_t=sc.wt, wrap_r=sc.wr)
    c = []
    for px, py in ((qx, qy), (qx + 1, qy), (qx, qy + 1), (qx + 1, qy + 1)):
        x = (px + 0.5) / W * 2.0 - 1.0; y = (py + 0.5) / H * 2.0 - 1.0
        s = sc.off[0] + x * sc.mul[0]; t = sc.off[1] + y * sc.mul[1]; r = sc.off[2] + x * sc.mul[2] + y * sc.mul[3]
        c.append(tuple(tr.rnd_float(f, q * (1.0 + nudge)) for q in (s, t, r)) + (0,))
    res = tr.sample(d, v.mem, dict(samp=0, mode=0, c=c), f)
    return [tuple(int(round(float(fr.to_real(res[p * 4 + ch], f)) * 255.0)) for ch in range(4)) for p in range(4)]


def compare(data, sc, W, H, flip, nquads=120, seed=1):
    f = fr.FP24; rng = random.Random(seed); bad = n = 0; first = []
    tol = 3
    for _ in range(nquads):
        qx = rng.randrange(0, W - 2, 2); qy = rng.randrange(0, H - 2, 2)
        exp = model_quad(sc, f, qx, qy, W, H)
        alt = None
        for p, (px, py) in enumerate(((qx, qy), (qx + 1, qy), (qx, qy + 1), (qx + 1, qy + 1))):
            row = H - 1 - py if flip else py
            o = (row * W + px) * 4
            got = (data[o + 3], data[o + 2], data[o + 1], data[o])
            ok = all(abs(a - b) <= tol for a, b in zip(got, exp[p]))
            if not ok and sc.filt in (NEAREST, NEAREST_MIPN):           # the last bits of the shader's coordinates
                if alt is None: alt = [model_quad(sc, f, qx, qy, W, H, e) for e in (3e-5, -3e-5)]
                ok = any(all(abs(a - b) <= tol for a, b in zip(got, q[p])) for q in alt)
            n += 1
            if not ok:
                bad += 1
                if len(first) < 3: first.append(f"pixel ({px}, {py}) got {got} expected {exp[p]}")
    return n, bad, first


def write_png(path, data, W, H, flip):
    raw = bytearray()
    for y in range(H):
        row = y if flip else H - 1 - y
        raw.append(0)
        rowb = data[row * W * 4:(row + 1) * W * 4]
        raw += bytes(c for o in range(W) for c in (rowb[o * 4 + 3], rowb[o * 4 + 2], rowb[o * 4 + 1]))
    def chunk(t, b): return struct.pack(">I", len(b)) + t + b + struct.pack(">I", zlib.crc32(t + b))
    with open(path, 'wb') as f:
        f.write(b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', struct.pack(">IIBBBBB", W, H, 8, 2, 0, 0, 0)) +
                chunk(b'IDAT', zlib.compress(bytes(raw), 6)) + chunk(b'IEND', b''))


# ------------------------------------------------------------------------------------------------ the board
def setup(bd, W, H, wv, wf, names):
    quad = [(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)]
    idx = [0, 1, 2, 0, 2, 3]
    bd.create_buffer(H_VBO, A_VBO, 4096); bd.write_buffer(H_VBO, 0, b''.join(struct.pack("<I", f24(c)) for p in quad for c in p))
    bd.create_buffer(H_IBO, A_IBO, 4096); bd.write_buffer(H_IBO, 0, struct.pack("<%dH" % len(idx), *idx))
    bd.create_buffer(H_VSPROG, A_VSPROG, 4096); bd.write_buffer(H_VSPROG, 0, words64_bytes(wv))
    bd.vs_program(H_VSPROG, 0, len(wv))
    for i in range(16):
        if i == 0: bd.vertex_attr(0, True, 2, H_VBO, 8, 0, type=fp.AT_FP24)
        else: bd.vertex_attr(i, False, 4, 0, 0, 0, (0.0, 0.0, 0.0, 1.0))
    bd.vs_state((W * 8.0, W * 8.0, H * 8.0, H * 8.0), cull=0, front_cw=0, fsmask=0x01, width=W, height=H, draw_id=DRAW_ID)
    bd.create_buffer(H_FSPROG, B_FSPROG, 65536); bd.write_buffer(H_FSPROG, 0, words64_bytes(wf))
    for h, base in zip(H_IMG, B_IMG):
        bd.create_image(h, base, W, H, fp.FMT_RGBA8, origin=1, usage=fp.USE_RT | fp.USE_SCANOUT | fp.USE_TRANSFER)
    t0 = time.monotonic(); base = B_TEX; nb = 0
    for k, name in enumerate(names):
        v = VOLS[name]
        nb += v.upload(bd, H_VOL + k, base)
        base += (v.chain * 4 + 0xFFFFF) & ~0xFFFFF
        bd.fence(timeout=60.0)
    print(f"volumes: {nb / 1e6:.1f} MB in {time.monotonic() - t0:.1f} s")
    errs = bd.take_errors()
    for e in errs: print("  EV_ERROR seq %d: %s" % (e[0], err_text(e[:4])))
    return len(idx), not errs


def draw_one(bd, W, H, vconst, nidx, i=0):
    seq = render_frame(bd, i, W, H, 1.0, 0.0, vconst, nidx)
    bd.wait_event(seq, timeout=20.0)


def run_check(bd, a, mode):
    wv, wf, vconst, fconst, funi, fs_cb, samplers = compile_programs('tex3d')
    unit = samplers['uVol']
    bd.set_mode(mode)
    W, H = MODE_SIZE[mode]
    all_sc = scenes()
    if '--scene' in a:
        all_sc = [s for s in all_sc if s.name == a[a.index('--scene') + 1]]
    nidx, ok = setup(bd, W, H, wv, wf, list(VOLS))
    if not ok: return 1
    fails = 0; flip = None; last = None
    for sc in all_sc:
        set_scene(bd, sc, wf, fconst, funi, fs_cb, unit)
        bd.fence(timeout=10.0)
        draw_one(bd, W, H, vconst, nidx)
        data = bd.mem_read(B_IMG[0], W * H * 4, board=1); last = data
        if flip is None:                                                # origin 1: find the row order once
            r = [compare(data, sc, W, H, q, nquads=20)[1] for q in (False, True)]
            flip = r[1] < r[0]
            print(f"rows {'top-down' if flip else 'bottom-up'} ({r[0]} / {r[1]} wrong)")
        n, bad, first = compare(data, sc, W, H, flip)
        errs = bd.take_errors()
        v = VOLS[sc.vol]
        print(f"{'ok  ' if bad == 0 and not errs else 'FAIL'} {sc.name:14s} {v.name} {v.w}x{v.h}x{v.dep}: {n} pixels, {bad} wrong")
        for s in first: print("     " + s)
        for e in errs: print("     EV_ERROR seq %d: %s" % (e[0], err_text(e[:4])))
        fails += (bad != 0) or bool(errs)
    if '--scene' not in a:
        # the check must be able to fail: the wrapped ramp against the model of another wrap mode
        sc = next(s for s in all_sc if s.name == 'r_mirror')
        set_scene(bd, sc, wf, fconst, funi, fs_cb, unit); bd.fence(timeout=10.0); draw_one(bd, W, H, vconst, nidx)
        data = bd.mem_read(B_IMG[0], W * H * 4, board=1)
        n, bad, _ = compare(data, sc._replace(wr=REPEAT), W, H, flip)
        seen = bad > n / 4
        print(f"{'ok  ' if seen else 'FAIL'} negative check: a mirrored ramp against the repeat model, {bad} of {n} wrong (must be many)")
        fails += not seen
        # what the command processor must refuse
        bd.take_errors()
        bd.create_image(H_VOL + 16, B_TEX, 300, 8, RGBA8, layers=8, levels=1, vol=True, usage=fp.USE_SAMPLED | fp.USE_TRANSFER)
        bd.fence(timeout=5.0); e1 = bd.take_errors()
        px = bytes(4)
        bd.write_image(VOLS['g64'].handle, px, fp.TF_RGBA8888, 0, 0, 1, 1, level=1, layer=15)   # level 1 of 16 has 8 slices
        bd.fence(timeout=5.0); e2 = bd.take_errors()
        good = len(e1) == 1 and len(e2) == 1
        print(f"{'ok  ' if good else 'FAIL'} refused: a 300-wide volume ({len(e1)} error), slice 15 of level 1 ({len(e2)} error)")
        fails += not good
    if '--png' in a and last is not None:
        write_png(a[a.index('--png') + 1], last, W, H, flip)
    print(f"RESULT fb_tex3d: {len(all_sc)} scenes, {fails} failed")
    return 1 if fails else 0


PERF = (
    ('v128_nearest',   'g128', NEAREST,   (0.45, 0.45, 0.4, 0.0)),
    ('v128_bilinear',  'g128', BILINEAR,  (0.45, 0.45, 0.4, 0.0)),
    ('v128_mipnearest','g128', MIP_NEAREST, (6.25, 4.7, 0.0, 0.0)),   # one level: lambda 1
    ('v128_trilinear', 'g128', TRILINEAR, (8.84, 6.63, 0.0, 0.0)),    # lambda 1.5: two levels, two slices each, 16 jobs per quad
)


def run_perf(bd, secs, mode=1):
    wv, wf, vconst, fconst, funi, fs_cb, samplers = compile_programs('tex3d')
    unit = samplers['uVol']
    bd.set_mode(mode)
    W, H = MODE_SIZE[mode]
    nidx, ok = setup(bd, W, H, wv, wf, ['g128'])
    if not ok: return 1
    for name, vol, filt, mul in PERF:
        sc = Scene(name, vol, filt, REPEAT, REPEAT, CLAMP, (0.5, 0.5, 0.5), mul)
        set_scene(bd, sc, wf, fconst, funi, fs_cb, unit)
        bd.fence(timeout=10.0)
        draw_one(bd, W, H, vconst, nidx)
        pending = collections.deque(); i = -1; t0 = time.monotonic()
        while time.monotonic() - t0 < secs:
            i += 1
            while len(pending) >= 2: bd.wait_event(pending.popleft(), timeout=20.0)
            pending.append(render_frame(bd, i, W, H, 1.0, 0.0, vconst, nidx))
        while pending: bd.wait_event(pending.popleft(), timeout=20.0)
        el = time.monotonic() - t0
        bd.fence(timeout=30.0)
        st = bd.status(); q = max(1, st['render_quads']); n = i + 1
        print(f"RESULT {name} mode={mode} {W}x{H} fps={n / el:.2f} cyc_per_quad={st['b_cyc_frame'] / q:.1f} "
              f"fs_busy={100.0 * st['b_cyc_fs'] / max(1, st['b_cyc_frame']):.1f}%")
        sys.stdout.flush()
        errs = bd.take_errors()
        for e in errs: print("  EV_ERROR seq %d: %s" % (e[0], err_text(e[:4])))
    return 0


def main():
    a = sys.argv[1:]
    if '--list' in a:
        for s in scenes(): print(s.name, s.vol)
        return 0
    ip = a[a.index('--ip') + 1] if '--ip' in a else fp.BOARD_IP
    mode = int(a[a.index('--mode') + 1]) if '--mode' in a else 0
    bd = fp.Board(ip)
    bd.hello()
    st = bd.status()
    lfl = fp.decode_link_flags(st['link_flags'])
    if not (lfl['trained'] and lfl['b_alive']): raise fp.ProtoError("the board link is not up / B is not alive")
    if '--perf' in a: return run_perf(bd, float(a[a.index('--secs') + 1]) if '--secs' in a else 4.0)
    return run_check(bd, a, mode)


if __name__ == '__main__': sys.exit(main())
