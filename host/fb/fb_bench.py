#!/usr/bin/env python3
"""fb_bench — the standard rates of the two boards, each measured alone, so they can be set beside published GPU figures:
pixel fill, depth test and rejection, blending, texturing, shader arithmetic, triangles and vertices, transfers. Every test
renders frames as fast as it can for a few seconds; the rate is what got through per second of wall clock, flush and host
included, and B's cycle counters show its own part of the frame.

  python3 host/fb/fb_bench.py [--ip 192.168.1.10] [--mode 1] [--secs 3] [--only NAME[,NAME]] [--list] [--no-zc] [--no-lazy]
                              [--no-skip]

Board A's coarse depth (the draws' flags from their DRAW_STATE, fbproto.zc_flags) and lazy TRI items are on, as in the
libraries: --no-zc sends no flags (A culls nothing), --no-lazy sends every TRI item; A's counters print per test (STATUS
layout 12). The partial_* tests draw into a few bands of a target they do not clear (as a desktop's redraw does) and
flush_empty flushes no draw and no clear: their FLUSHes leave the other bands out (FLUSH flags bit 10, as the libraries do on
boards that report it), --no-skip walks every band.

Fill and texture tests draw L full-screen layers per frame, triangle tests a grid of triangles of a given area over the
whole screen with an MVP transform (tri_culled: the same grid with both faces culled, so geometry and no pixels). Shaders
are compiled with forward=True, as host/win/shc always does. Standard library + the repository's tools.
"""
import collections, math, os, struct, sys, time
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(HERE, '..', '..', 'tools'))
import fbproto as fp
import glslc, isa64
from fb_image import MODE_SIZE
from fb_shader import (words64_bytes, f24, H_VBO, H_IBO, H_VSPROG, A_VBO, A_IBO, A_VSPROG, H_FSPROG, B_FSPROG,
                       H_IMG, B_IMG, DRAW_ID, err_text)
from fb_texperf import upload_texture, FILT, B_TEX

CLEAR = (0.0, 0.0, 0.0, 1.0)
H_T = tuple(0xC060 + k for k in range(8))                       # 8 textures, 256 x 256 RGBA8, full chains
TSIZE = 256
SRC_ALPHA, ONE_MINUS_SRC_ALPHA, ONE = 6, 7, 1

VS_FILL = """attribute vec3 aPos;
uniform vec4 uS;
varying vec4 vUV;
void main() { gl_Position = vec4(aPos, 1.0); vUV = vec4(aPos.x * uS.x, aPos.y * uS.y, 0.0, 1.0); }
"""
VS_GEOM = """attribute vec3 aPos;
uniform mat4 uM;
void main() { gl_Position = uM * vec4(aPos, 1.0); }
"""
FS_FLAT = "precision mediump float;\nuniform vec4 uC;\nvoid main() { gl_FragColor = uC; }\n"
FS_VARY = "precision mediump float;\nvarying vec4 vUV;\nvoid main() { gl_FragColor = vUV; }\n"


def fs_tex(n):
    s = "precision mediump float;\n" + "".join(f"uniform sampler2D s{k};\n" for k in range(n)) + "varying vec4 vUV;\nvoid main() {\n"
    s += "    vec4 c = texture2D(s0, vUV.xy);\n" + "".join(f"    c += texture2D(s{k}, vUV.xy);\n" for k in range(1, n))
    return s + (f"    gl_FragColor = c * {1.0 / n:.6f};\n}}\n" if n > 1 else "    gl_FragColor = c;\n}\n")


def fs_alu(n):
    """n vec4 multiply-adds on run-time values: 4 n scalar multiply-adds per pixel"""
    return ("precision mediump float;\nuniform vec4 uA;\nuniform vec4 uB;\nvarying vec4 vUV;\nvoid main() {\n    vec4 v = vUV;\n" +
            "    v = v * uA + uB;\n" * n + "    gl_FragColor = v;\n}\n")


Test = collections.namedtuple('Test', 'name kind fs layers z blend filt lod nsamp tri alu')
def T(name, kind, fs, layers=1, z=None, blend=False, filt='bilinear', lod=0.0, nsamp=0, tri=0, alu=0):
    return Test(name, kind, fs, layers, z, blend, filt, lod, nsamp, tri, alu)

TESTS = [
    T('fill_flat',      'fill', FS_FLAT, 6),
    T('fill_varying',   'fill', FS_VARY, 6),
    T('fill_depth_pass', 'fill', FS_FLAT, 6, z='pass'),
    T('fill_depth_reject', 'fill', FS_FLAT, 12, z='reject'),
    T('fill_blend',     'fill', FS_FLAT, 4, blend=True),
    T('tex1_nearest',   'fill', fs_tex(1), 2, filt='nearest', nsamp=1),
    T('tex1_bilinear',  'fill', fs_tex(1), 2, nsamp=1),
    T('tex1_trilinear', 'fill', fs_tex(1), 2, filt='trilinear', lod=1.5, nsamp=1),
    T('tex2_bilinear',  'fill', fs_tex(2), 2, nsamp=2),
    T('tex4_bilinear',  'fill', fs_tex(4), 1, nsamp=4),
    T('tex8_bilinear',  'fill', fs_tex(8), 1, nsamp=8),
    T('alu_16',         'fill', fs_alu(16), 1, alu=16),
    T('alu_64',         'fill', fs_alu(64), 1, alu=64),
    T('tri_culled',     'geom', FS_FLAT, tri=8),
    T('tri_8px',        'geom', FS_FLAT, tri=8),
    T('tri_32px',       'geom', FS_FLAT, tri=32),
    T('tri_128px',      'geom', FS_FLAT, tri=128),
    T('tri_512px',      'geom', FS_FLAT, tri=512),
    T('partial_2bands', 'part', FS_FLAT, tri=16),               # (tri: the rows drawn, about the target's middle)
    T('partial_8bands', 'part', FS_FLAT, tri=64),
    T('flush_empty',    'part', FS_FLAT, tri=0),
]


def rows_mesh(H, rows):
    """a full-width quad over `rows` rows about the target's middle"""
    y = rows / H
    return [(-1.0, -y, 0.0), (1.0, -y, 0.0), (1.0, y, 0.0), (-1.0, y, 0.0)], [0, 1, 2, 0, 2, 3]


def compile_pair(vs, fs):
    rv = glslc.compile_stage(vs, 'vs', isa64=True, forward=True, noops=NOOPS['vs'])
    rf = glslc.compile_stage(fs, 'fs', isa64=True, forward=True, noops=NOOPS['fs'])
    wv, _ = isa64.assemble(rv['sasm'].splitlines(), vs=True)
    wf, _ = isa64.assemble(rf['sasm'].splitlines())
    return rv, rf, wv, wf


def uniforms(r, values):
    """the program's uniform vectors from {name: [vec4, ...]}, then the compiler's constants"""
    u = [(0.0, 0.0, 0.0, 0.0)] * r['const_base']
    for name, vs in values.items():
        if name in r['uniforms']:
            for k, v in enumerate(vs): u[r['uniforms'][name][0] + k] = v
    return u + [tuple(float(c) for c in k) for k in r['const_pool']]


def layer_mesh(L, z):
    """L full-screen quads in drawing order"""
    v = []; idx = []
    for k in range(L):
        d = 0.0 if z is None else (0.9 - 1.8 * k / max(1, L - 1)) if z == 'pass' else (-0.9 + 1.8 * k / max(1, L - 1))
        b = len(v)
        v += [(-1.0, -1.0, d), (1.0, -1.0, d), (1.0, 1.0, d), (-1.0, 1.0, d)]
        idx += [b, b + 1, b + 2, b, b + 2, b + 3]
    return v, idx


def grid_mesh(W, H, area):
    """triangles of `area` pixels covering the screen: a grid of cells of 2 area pixels"""
    side = math.sqrt(2.0 * area)
    c = max(1, round(W / side)); r = max(1, round(H / side))
    v = [(-1.0 + 2.0 * i / c, -1.0 + 2.0 * j / r, 0.0) for j in range(r + 1) for i in range(c + 1)]
    idx = []
    for j in range(r):
        for i in range(c):
            a = j * (c + 1) + i; b = a + 1; d = a + c + 1; e = d + 1
            idx += [a, b, e, a, e, d]
    return v, idx


def run_test(bd, t, W, H, secs, textures_up):
    vs = VS_GEOM if t.kind == 'geom' else VS_FILL
    rv, rf, wv, wf = compile_pair(vs, t.fs)
    v, idx = grid_mesh(W, H, t.tri) if t.kind == 'geom' else rows_mesh(H, max(t.tri, 1)) if t.kind == 'part' else layer_mesh(t.layers, t.z)
    bd.create_buffer(H_VBO, A_VBO, 4 << 20); bd.write_buffer(H_VBO, 0, b''.join(struct.pack("<I", f24(c)) for p in v for c in p))
    bd.create_buffer(H_IBO, A_IBO, 1 << 20); bd.write_buffer(H_IBO, 0, struct.pack("<%dH" % len(idx), *idx))
    bd.create_buffer(H_VSPROG, A_VSPROG, 4096); bd.write_buffer(H_VSPROG, 0, words64_bytes(wv))
    bd.vs_program(H_VSPROG, 0, len(wv))
    for i in range(16):
        if i == 0: bd.vertex_attr(0, True, 3, H_VBO, 12, 0, type=fp.AT_FP24)
        else: bd.vertex_attr(i, False, 4, 0, 0, 0, (0.0, 0.0, 0.0, 1.0))
    mask = 0
    for name, (slot, _ty) in rf['varyings'].items(): mask |= 1 << slot
    bd.create_buffer(H_FSPROG, B_FSPROG, 65536); bd.write_buffer(H_FSPROG, 0, words64_bytes(wf))
    for h_, base in zip(H_IMG, B_IMG):
        bd.create_image(h_, base, W, H, fp.FMT_RGBA8, origin=1, usage=fp.USE_RT | fp.USE_SCANOUT | fp.USE_TRANSFER)
    if t.nsamp and not textures_up:
        t0 = time.monotonic()
        for k in range(8): upload_texture(bd, H_T[k], B_TEX + k * 0x10_0000, TSIZE, fp.FMT_RGBA8, 9)
        bd.fence(timeout=60.0)
        textures_up.append(time.monotonic() - t0)
    ufs = uniforms(rf, {'uC': [(0.25, 0.5, 0.75, 0.5)], 'uA': [(0.9, 0.8, 0.7, 0.9)], 'uB': [(0.02, 0.03, 0.04, 0.01)]})
    h = [0] * 23
    h[0] = 0xF << 16
    if t.z: h[0] |= 1 | 1 << 1 | fp.FN_LESS << 2
    if t.blend:
        h[0] |= 1 << 6
        h[4] = SRC_ALPHA | ONE_MINUS_SRC_ALPHA << 4 | ONE << 8 | ONE_MINUS_SRC_ALPHA << 12
    h[11] = len(wf) << 16 | H_FSPROG; h[12] = 0; h[13] = len(ufs)
    for name, unit in rf['samplers'].items():
        h[15 + unit] = H_T[unit] | FILT[t.filt] << 16
    # board A's state, with the flags the draw state above promises (the program's DISCARD / STOZ, opcodes 29 / 21)
    ops = {(w >> 57) & 0x7F for w in wf}
    zc = (0 if NO_ZC else fp.zc_flags(h[0], discard=29 in ops, frag_depth=21 in ops)) | (0 if NO_LAZY else fp.ZC_LAZY)
    bd.vs_state((W * 8.0, W * 8.0, H * 8.0, H * 8.0), cull=3 if t.name == 'tri_culled' else 0, front_cw=0, fsmask=mask or 1,
                width=W, height=H, draw_id=DRAW_ID, zc=zc)
    bd.draw_state(DRAW_ID, h, ufs)
    bd.fence(timeout=10.0)
    errs = bd.take_errors()
    for e in errs: print("  EV_ERROR seq %d: %s" % (e[0], err_text(e[:4])))
    if errs: return None
    # 1 texel per pixel at lambda 0, 2^lod with mips: s per x = W / 2 / 256 x 2^lod
    k = 2.0 ** t.lod
    uvs = uniforms(rv, {'uS': [(W / 2.0 / TSIZE * k, H / 2.0 / TSIZE * k, 0.0, 0.0)],
                        'uM': [(1.0, 0.0, 0.0, 0.0), (0.0, 1.0, 0.0, 0.0), (0.0, 0.0, 1.0, 0.0), (0.0, 0.0, 0.0, 1.0)]})
    cc = tuple(fp.c16(c) for c in CLEAR)
    flags = fp.FL_CLEAR_COLOR | fp.FL_CMASK(0xF) | (fp.FL_CLEAR_DEPTH | fp.FL_CLEAR_STENCIL if t.z else 0)
    if t.kind == 'part': flags = 0 if NO_SKIP else fp.FL_SKIP_BANDS   # the target kept (no clear): every band walked is loaded and written
    pending = collections.deque(); i = -1; t0 = time.monotonic(); st0 = bd.status()
    while time.monotonic() - t0 < secs:
        i += 1
        while len(pending) >= 2: bd.wait_event(pending.popleft(), timeout=20.0)
        with bd.batch():
            bd.frame_begin()
            bd.vs_uniforms(0, uvs)
            if t.kind != 'part' or t.tri: bd.draw(0, len(idx), 0, index_handle=H_IBO)
            bd.flush(H_IMG[i & 1], flags, cc, clear_depth=0xFFFFFF, scissor=(0, 0, W, H))
        pending.append(bd.present(H_IMG[i & 1], frame_id=i, hold=False, wait=False))
    while pending: bd.wait_event(pending.popleft(), timeout=20.0)
    el = time.monotonic() - t0
    bd.fence(timeout=30.0)
    st = bd.status()
    for e in bd.take_errors(): print("  EV_ERROR seq %d: %s" % (e[0], err_text(e[:4])))
    n = i + 1; fps = n / el
    bms = st['b_cyc_frame'] / fp.CLK_KHZ
    ams = ((st['a_cyc_geom'] - st0['a_cyc_geom']) & 0xFFFFFFFF) / fp.CLK_KHZ / n   # a running count: its share per frame
    rate = ""
    px = W * H
    if t.kind == 'geom':
        tris = len(idx) // 3
        rate = f"tris_M={tris * fps / 1e6:.3f} verts_M={len(v) * fps / 1e6:.3f} tris={tris}"
    elif t.kind == 'part':
        rate = f"rows={t.tri} skip={0 if NO_SKIP else 1}"
        if st.get('layout', 0) >= 13: rate += f" bands_left_out={((st['a_bands_skipped'] - st0['a_bands_skipped']) & 0xFFFFFFFF) / n:.1f}/frame"
    else:
        rate = f"Mpix={t.layers * px * fps / 1e6:.1f}"
        if t.nsamp: rate += f" Msamples={t.layers * px * fps * t.nsamp / 1e6:.1f}"
        if t.alu: rate += f" Mscalar_ops={t.layers * px * fps * 4 * t.alu / 1e6:.0f}"
    if st.get('layout', 0) >= 9: rate += f" a_visits={((st['a_visits'] - st0['a_visits']) & 0xFFFFFFFF) / n:.0f}/frame"
    print(f"RESULT {t.name} fps={fps:.2f} {rate} b_frame_ms={bms:.2f} a_geom_ms={ams:.2f} instr={len(wf)} vs_instr={len(wv)} frames={n} | " + fp.b_frame_split(st))
    if st.get('layout', 0) >= 10: print(f"  PROFILE {t.name} " + fp.a_geom_split(st, st0, n))
    if st.get('layout', 0) >= 12: print(f"  CZ {t.name} " + fp.a_cz_split(st, st0, n))
    sys.stdout.flush()
    return fps


def transfers(bd):
    """texture upload and readback, MB/s"""
    data = bytes(2048 * 2048 * 4)
    bd.create_image(0xC070, 0x3400_0000, 2048, 2048, fp.FMT_RGBA8, usage=fp.USE_SAMPLED | fp.USE_TRANSFER)
    t0 = time.monotonic()
    bd.write_image(0xC070, data, fp.TF_RGBA8888, 0, 0, 2048, 2048)
    bd.fence(timeout=60.0)
    up = len(data) / 1e6 / (time.monotonic() - t0)
    t0 = time.monotonic()
    got = bd.mem_read(0x3400_0000, 4 << 20, board=1)
    down = len(got) / 1e6 / (time.monotonic() - t0)
    print(f"RESULT transfer upload_MBps={up:.1f} readback_MBps={down:.1f}")


NO_ZC = NO_LAZY = NO_SKIP = False
NOOPS = {'vs': 0, 'fs': 0}                                     # fbproto.stage_noops of the board (main sets it)


def main():
    global NO_ZC, NO_LAZY, NO_SKIP, NOOPS
    a = sys.argv[1:]
    def opt(name, default, conv=str): return conv(a[a.index(name) + 1]) if name in a else default
    NO_ZC, NO_LAZY, NO_SKIP = '--no-zc' in a, '--no-lazy' in a, '--no-skip' in a
    if '--list' in a:
        for t in TESTS: print(t.name)
        return 0
    ip = opt('--ip', fp.BOARD_IP); mode = opt('--mode', 1, int); secs = opt('--secs', 3.0, float)
    only = opt('--only', None); names = only.split(',') if only else None
    bd = fp.Board(ip)
    bd.hello()
    st = bd.status()
    lfl = fp.decode_link_flags(st['link_flags'])
    if not (lfl['trained'] and lfl['b_alive']): raise fp.ProtoError("the board link is not up / B is not alive")
    bd.set_mode(mode)
    W, H = MODE_SIZE[mode]
    NOOPS = fp.stage_noops(st)                                 # the ops the board's cores lack are lowered by the compiler
    print(f"board A build {st['build']}, {W} x {H}, {secs} s per test{fp.noops_note(NOOPS)}")
    up = []
    for t in TESTS:
        if names is None or t.name in names:
            run_test(bd, t, W, H, secs, up)
    if names is None: transfers(bd)
    return 0


if __name__ == '__main__': sys.exit(main())
