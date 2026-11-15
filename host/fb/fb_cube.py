#!/usr/bin/env python3
"""fb_cube — rotating, lit cubes rendered by the two boards and shown on board B's HDMI output: the 3D test that is
verified ON SCREEN (render-then-PRESENT), and the performance scene (`--cubes N`).

  python3 host/fb/fb_cube.py [--ip 192.168.1.10] [--mode M] [--frames N | --secs S] [--speed DEG] [--cubes N] [--draws]
                             [--nohold] [--nobatch] [--scale S] [--check] [--ppm] [--strict] [--selftest]

Scene: N cubes (default 32) on a square grid, every one turning about y and 0.7 x that about x with its own phase, one
directional light (Lambert + ambient per vertex, per-face normals: flat faces), 6 face colours; depth test, no blend,
back faces culled. Two ways to draw it:
  default      ONE draw for every cube: tests/cubes.vert computes each vertex's rotation from the global angle + the
               cube's phase (attribute 3) with the hardware SIN / COS — 24 N vertices, 12 N triangles, 71 instructions
  --draws      one draw per cube with tests/cube.vert (MVP and rotation as row uniforms, 40 instructions): per-draw
               VS_UNIFORMS + DRAW, all draws under one draw id (B keeps one state); measures A's per-draw cost
  --forward    compile the shaders with forward=True (block-local copy forwarding + dead stores), as the C compiler
               host/win/shc always does; the default is the tools' plain compile (the two are compared on the hardware)
  --bones      one draw for every cube with tests/cubes_bones.vert: every cube's model matrix is a uniform (mat4
               uBone[N]) picked PER VERTEX with a run-time index (the cube number in attribute 3's .w) — the ES2
               vertex-shader uniform indexing (LDPX); N <= ~60 (256 uniform slots)
A frame's commands (FRAME_BEGIN, the uniforms + draws, the FLUSH) go out in BATCH datagrams (fbproto Board.batch():
~6 cubes' draws per datagram instead of 2 datagrams per cube); `--nobatch` sends one command per datagram.
Board B runs tests/cube.frag (the interpolated colour) with depth test; the FLUSH clears colour + depth + stencil (the
lazy fast clear: no band load); the frame goes into one of two RGBA8 images of the mode's size (GL origin) and is
PRESENTed with the hold flag (one frame per vsync, no tearing). `--nohold` presents at the next vsync without holding
B's command processor: frames then run as fast as the GPU renders them (tearing allowed) = the throughput measure. The
host keeps two frames in flight; every 2 s it prints fps, quads and triangle records per frame (A's counters).

A board-link retrain aborts the frame in flight (A's dispatcher: ERR_RENDER with the frame's age as detail — the 3 s
RENDER_TIMEOUT is the other cause); the run counts those and continues (`--strict`: stop at the first error).
  --check   after the first frame (a fixed rotation) read the target back from B's DDR and compare the centre of every
            front face and 5 points inside it with the host's lighting, and the clear colour in the corners, +-2 per
            channel (the hardware lights at FP24; the sin / cos of cubes.vert are the hardware's, <= 4e-6)
  --ppm     write that frame to tools/gen/cube_frame.ppm (top row first)
  --selftest  compile the shaders and run them through tools/isa64.py's core simulator against the host's maths (no board)
Standard library + this repository's tools (glslc, isa64, float_ref).
"""
import collections, contextlib, math, os, struct, sys, time
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(HERE, '..', '..', 'tools'))
import fbproto as fp
import glslc, isa64, float_ref as fr
from fb_image import MODE_SIZE

TESTS = os.path.join(HERE, '..', '..', 'tests')
GEN = os.path.join(HERE, '..', '..', 'tools', 'gen')

# ---- resources: A's host heap (384 MB..) and B's (256 MB..; fb_video's images are at 0x2800_0000)
H_VBO, H_IBO, H_VSPROG = 0x0021, 0x0022, 0x0023                 # board A buffers
A_VBO, A_IBO, A_VSPROG = 0x1A00_0000, 0x1A40_0000, 0x1A50_0000  # byte addresses in A's heap (VBO up to 4 MB)
H_FSPROG, B_FSPROG = 0x8021, 0x1000_0000                        # board B buffer (bit 15), byte address in B's heap
H_IMG = (0xC030, 0xC031)                                        # the two render targets / scanout images
B_IMG = (0x3000_0000, 0x3040_0000)                              # 4 MB each: up to 1280 x 800 x 4
DRAW_ID = 1
MAX_CUBES = 2000                                                # 48 000 vertices: 1 draw < 65 536, VBO 48 000 x 13 words
RENDER_TIMEOUT_CYCLES = 300_000_000                             # fb_dispatch_a RENDER_TIMEOUT (3 s at 100 MHz)

# ---- the scene
FOV_DEG, NEAR, FAR, CAM_Z = 45.0, 1.0, 40.0, 4.0
START_DEG = 35.0                                                # frame 0 shows three faces of cube 0 (front, left, top)
X_RATE = 0.7                                                    # the x rotation = X_RATE x the y rotation
PHASE_STEP = 0.37                                               # radians between consecutive cubes' phases
LIGHT = (0.4, 0.6, 0.7)                                         # toward the light, world space (normalised below)
AMBIENT = 0.25
CLEAR = (0.05, 0.05, 0.12, 1.0)
FACES = [   # (normal, colour, the 4 corners CCW seen from outside)
    ((0, 0, 1),  (0.25, 0.45, 1.0), [(-1, -1, 1), (1, -1, 1), (1, 1, 1), (-1, 1, 1)]),
    ((0, 0, -1), (1.0, 0.9, 0.2),   [(1, -1, -1), (-1, -1, -1), (-1, 1, -1), (1, 1, -1)]),
    ((1, 0, 0),  (1.0, 0.25, 0.25), [(1, -1, 1), (1, -1, -1), (1, 1, -1), (1, 1, 1)]),
    ((-1, 0, 0), (0.2, 0.9, 0.9),   [(-1, -1, -1), (-1, -1, 1), (-1, 1, 1), (-1, 1, -1)]),
    ((0, 1, 0),  (0.3, 0.9, 0.3),   [(-1, 1, 1), (1, 1, 1), (1, 1, -1), (-1, 1, -1)]),
    ((0, -1, 0), (0.9, 0.3, 0.9),   [(-1, -1, -1), (1, -1, -1), (1, -1, 1), (-1, -1, 1)]),
]


# ------------------------------------------------------------------------------------------------ maths (row-major 4 x 4)
def mat_mul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def mat_vec(m, v):
    return [sum(m[i][k] * v[k] for k in range(4)) for i in range(4)]


def rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return [[1, 0, 0, 0], [0, c, -s, 0], [0, s, c, 0], [0, 0, 0, 1]]


def rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return [[c, 0, s, 0], [0, 1, 0, 0], [-s, 0, c, 0], [0, 0, 0, 1]]


def perspective(fov_deg, aspect, n, f):
    t = 1.0 / math.tan(math.radians(fov_deg) / 2)
    return [[t / aspect, 0, 0, 0], [0, t, 0, 0], [0, 0, (f + n) / (n - f), 2 * f * n / (n - f)], [0, 0, -1, 0]]


def normalize(v):
    l = math.sqrt(sum(x * x for x in v))
    return [x / l for x in v]


class Scene:
    """N cubes on a g x g grid (g = ceil(sqrt N)), spacing 2.6 cube units, scaled by 1 / g so the grid fills the same
    view as one cube; cube k at grid cell (k % g, k // g) with phase k * PHASE_STEP"""
    def __init__(self, n, aspect, size=1.0):
        self.n = n; self.g = max(1, math.ceil(math.sqrt(n))); self.scale = size / self.g
        self.centres = [((k % self.g - (self.g - 1) / 2) * 2.6 * self.scale, ((self.g - 1) / 2 - k // self.g) * 2.6 * self.scale, 0.0)
                        for k in range(n)]
        self.phases = [k * PHASE_STEP for k in range(n)]
        V = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, -CAM_Z], [0, 0, 0, 1]]
        self.vp = mat_mul(perspective(FOV_DEG, aspect, NEAR, FAR), V)

    def angle(self, frame, speed_deg):
        return math.radians(START_DEG + frame * speed_deg)

    def rotation(self, a, k):
        """R = Ry(a + phase) * Rx(X_RATE (a + phase)) of cube k"""
        t = a + self.phases[k]
        return mat_mul(rot_y(t), rot_x(X_RATE * t))

    def model(self, R, k):
        """the model matrix: scale, rotate, translate to the centre"""
        c = self.centres[k]; s = self.scale
        return [[R[i][j] * s if j < 3 else c[i] for j in range(4)] if i < 3 else [0, 0, 0, 1] for i in range(4)]

    def mvp(self, R, k):
        return mat_mul(self.vp, self.model(R, k))


def face_colour(R, face):
    """the lit colour of a face as the vertex shaders compute it (all 4 corners share the normal: flat)"""
    n = mat_vec(R, list(face[0]) + [0])[:3]
    d = max(sum(a * b for a, b in zip(n, normalize(LIGHT))), 0.0)
    return [c * (AMBIENT + (1 - AMBIENT) * d) for c in face[1]]


def rgba8(c):
    """the stored RGBA8 word (0xRRGGBBAA) of a colour: c16 = round(c * 4080), no dither -> floor(c16 / 16)"""
    ch = [fp.c16(x) >> 4 for x in list(c) + [1.0]]
    return ch[0] << 24 | ch[1] << 16 | ch[2] << 8 | ch[3]


# ------------------------------------------------------------------------------------------------ geometry and programs
def cube_buffers(scene, instanced, bones=False):
    """-> (vertex words, index words, stride): per vertex pos3 nrm3 col3 [+ inst4 = centre, phase (bones: the cube
    number)], FP24 in the low 24 bits; indices u16. instanced: every cube's 24 vertices; else one cube (the draws
    place it with uniforms)"""
    vw = []; idx = []
    ncubes = scene.n if instanced else 1
    for k in range(ncubes):
        inst = [fp.fp24(float(x)) for x in scene.centres[k]] + [fp.fp24(float(k) if bones else scene.phases[k])]
        for f, (n, col, corners) in enumerate(FACES):
            for p in corners:
                vw += [fp.fp24(float(x)) for x in p] + [fp.fp24(float(x)) for x in n] + [fp.fp24(float(x)) for x in col]
                if instanced: vw += inst
            b = 24 * k + 4 * f
            idx += [b, b + 1, b + 2, b, b + 2, b + 3]
    return vw, idx, 13 if instanced else 9


def compile_programs(instanced, bones=False, ncubes=0, forward=False, noops=None):
    """noops: fbproto.stage_noops of the board (the units its cores lack), None = both cores complete"""
    noops = noops or {'vs': 0, 'fs': 0}
    vsrc = open(os.path.join(TESTS, 'cubes_bones.vert' if bones else 'cubes.vert' if instanced else 'cube.vert')).read()
    if bones: vsrc = f"#define NB {ncubes}\n" + vsrc                # the bone array's size = the cube count
    rv = glslc.compile_stage(vsrc, 'vs', isa64=True, forward=forward, noops=noops['vs'])
    rf = glslc.compile_stage(open(os.path.join(TESTS, 'cube.frag')).read(), 'fs', isa64=True, forward=forward, noops=noops['fs'])
    wv, _ = isa64.assemble(rv['sasm'].splitlines(), vs=True)
    wf, _ = isa64.assemble(rf['sasm'].splitlines())
    att = {'aPos': (0, 'vec3'), 'aNormal': (1, 'vec3'), 'aColor': (2, 'vec3')}
    if instanced: att['aInst'] = (3, 'vec4')
    assert rv['attribs'] == att, rv['attribs']
    names = ('uRow0', 'uRow1', 'uRow2', 'uRow3', 'uLight', 'uAnim') if instanced else \
            ('uRow0', 'uRow1', 'uRow2', 'uRow3', 'uNRow0', 'uNRow1', 'uNRow2', 'uLight')
    assert [rv['uniforms'][k][0] for k in names] == list(range(len(names)))
    nuser = len(names)
    if bones:
        assert rv['uniforms']['uBone'] == (nuser, 'mat4', ncubes), rv['uniforms']['uBone']
        nuser += 4 * ncubes
        if nuser + len(rv['const_pool']) > 256: raise SystemExit(f"--bones: {ncubes} cubes need {nuser + len(rv['const_pool'])} uniform slots (256)")
    assert rv['const_base'] == nuser
    assert rv['varyings'] == {'vColor': (0, 'vec4')} and rf['varyings'] == {'vColor': (0, 'vec4')}
    vconst = [tuple(float(c) for c in k) for k in rv['const_pool']]
    fconst = [tuple(float(c) for c in k) for k in rf['const_pool']]
    return wv, wf, vconst, fconst, rf['const_base']


def vs_uniforms(scene, instanced, a, k, vconst, bones=False):
    """the vertex program's uniforms for angle a: instanced -> VP rows, light, (angle, scale, X_RATE, 0); bones -> VP
    rows, light, (angle, 1 / scale, X_RATE, 0), then every cube's model matrix as 4 COLUMNS (a GLSL mat4 uniform);
    per draw -> cube k's MVP rows, its rotation's rows, light; then the compiler's constants"""
    light = tuple(normalize(LIGHT)) + (AMBIENT,)
    if bones:
        u = [tuple(scene.vp[i]) for i in range(4)] + [light, (a, 1.0 / scene.scale, X_RATE, 0.0)]
        for kk in range(scene.n):
            M = scene.model(scene.rotation(a, kk), kk)
            u += [tuple(M[i][j] for i in range(4)) for j in range(4)]
    elif instanced:
        u = [tuple(scene.vp[i]) for i in range(4)] + [light, (a, scene.scale, X_RATE, 0.0)]
    else:
        R = scene.rotation(a, k); mvp = scene.mvp(R, k)
        u = [tuple(mvp[i]) for i in range(4)] + [tuple(R[i][:3]) + (0.0,) for i in range(3)] + [light]
    return [tuple(float(c) for c in v) for v in u + vconst]     # floats only: fbproto packs ints as FP24 bits


def words64_bytes(words):
    return b''.join(struct.pack("<II", w & 0xFFFFFFFF, w >> 32) for w in words)


def selftest(scene, instanced, wv, wf, vconst, fconst, fs_const_base, bones=False):
    """the programs through tools/isa64.py's warp simulator: positions and colours of every cube's 24 vertices at frame 0
    against the host's maths (FP24, hardware SIN / COS: within 2^-9 relative), the fragment program passing varying 0"""
    vw, idx, stride = cube_buffers(scene, instanced, bones)
    a = scene.angle(0, 1.0)
    one = fp.fp24(1.0); bad = 0; nvert = len(vw) // stride
    params = None
    for v0 in range(0, nvert, 16):
        lanes = list(range(v0, min(nvert, v0 + 16)))
        k0 = lanes[0] // 24
        if params is None or not instanced:
            uni = vs_uniforms(scene, instanced, a, k0, vconst, bones)
            params = [[fp.fp24(c) for c in v] for v in uni] + [[0] * 4] * (256 - len(uni))
        varin = []
        for v in lanes:
            w = vw[stride * v: stride * (v + 1)]
            rows = [w[0:3] + [one], w[3:6] + [one], w[6:9] + [one]] + ([w[9:13]] if instanced else [])
            varin.append(rows + [[0, 0, 0, one]] * (16 - len(rows)))
        varin += [varin[0]] * (16 - len(lanes))
        r = isa64.simulate_vs2(wv, isa64.FSInputs(params, varin, interp=False))
        assert not r.err, "vertex program: core error"
        for l, v in enumerate(lanes):
            k = v // 24 if instanced else k0
            if not instanced and v // 24 != k0: continue
            face = FACES[(v % 24) // 4]; p = face[2][v % 4]
            R = scene.rotation(a, k)
            exp_pos = mat_vec(scene.mvp(R, k), list(p) + [1.0])
            exp_col = face_colour(R, face) + [1.0]
            got_pos = [float(fr.to_real(x, fr.FP24)) for x in r.outc[l]]
            got_col = [float(fr.to_real(x, fr.FP24)) for x in r.outv[0][l]]
            for name, e, g in (('position', exp_pos, got_pos), ('colour', exp_col, got_col)):
                for c in range(4):
                    if abs(e[c] - g[c]) > 2 ** -9 * max(1.0, abs(e[c])):
                        bad += 1
                        if bad <= 8: print(f"  selftest: cube {k} vertex {v % 24} {name}.{'xyzw'[c]}: core {g[c]:.6f}, host {e[c]:.6f}")
    fparams = [[0] * 4] * fs_const_base + [[fp.fp24(c) for c in v] for v in fconst]
    fparams += [[0] * 4] * (64 - len(fparams))
    col = [[fp.fp24(0.1 * (p + 1)), fp.fp24(0.5), fp.fp24(0.25 * p), one] for p in range(4)]
    varin = [[col[p]] + [[0] * 4] * 15 for p in range(4)] + [[[0] * 4] * 16 for _ in range(12)]
    res = isa64.simulate_fs2(wf, isa64.FSInputs(fparams, varin, active=0xF, interp=False))
    for p in range(4):
        if res.outc[p] != col[p] or (res.killed >> p) & 1:
            bad += 1; print(f"  selftest: fragment program pixel {p}: {res.outc[p]} killed {(res.killed >> p) & 1}")
    print(f"selftest ({'one draw, cubes_bones.vert' if bones else 'one draw, cubes.vert' if instanced else 'a draw per cube, cube.vert'}): "
          f"vertex program {len(wv)} instructions, fragment program {len(wf)}, {nvert} vertices + 4 pixels through the core "
          f"simulator: {bad} mismatches")
    return bad == 0


# ------------------------------------------------------------------------------------------------ the board
def setup(bd, W, H, scene, instanced, wv, wf, vconst, fconst, fs_const_base, bones=False):
    vw, idx, stride = cube_buffers(scene, instanced, bones)
    # A: the buffers, the vertex program, the attribute descriptors (all 16: the table has no reset), the state
    bd.create_buffer(H_VBO, A_VBO, 4 << 20); bd.write_buffer(H_VBO, 0, b''.join(struct.pack("<I", w) for w in vw))
    bd.create_buffer(H_IBO, A_IBO, 1 << 20); bd.write_buffer(H_IBO, 0, struct.pack("<%dH" % len(idx), *idx))
    bd.create_buffer(H_VSPROG, A_VSPROG, 4096); bd.write_buffer(H_VSPROG, 0, words64_bytes(wv))
    bd.vs_program(H_VSPROG, 0, len(wv))
    nattr = 4 if instanced else 3
    for i in range(16):                                        # FP24 words: byte stride / offset = 4 x the word counts
        if i < nattr: bd.vertex_attr(i, True, 4 if i == 3 else 3, H_VBO, 4 * stride, 12 * i, type=fp.AT_FP24)
        else: bd.vertex_attr(i, False, 4, 0, 0, 0, (0.0, 0.0, 0.0, 1.0))
    bd.vs_state((W * 8.0, W * 8.0, H * 8.0, H * 8.0), cull=1, front_cw=0, fsmask=0x01, width=W, height=H, draw_id=DRAW_ID)
    # B: the fragment program, the two images, the draw state (depth LESS + write, no blend, colour mask RGBA)
    bd.create_buffer(H_FSPROG, B_FSPROG, 4096); bd.write_buffer(H_FSPROG, 0, words64_bytes(wf))
    for h, base in zip(H_IMG, B_IMG):
        bd.create_image(h, base, W, H, fp.FMT_RGBA8, origin=1, usage=fp.USE_RT | fp.USE_SCANOUT | fp.USE_TRANSFER)
    h = [0] * 23
    h[0] = 1 | 1 << 1 | fp.FN_LESS << 2 | 0xF << 16
    nuni = fs_const_base + len(fconst)
    h[11] = len(wf) << 16 | H_FSPROG; h[12] = 0; h[13] = nuni
    bd.draw_state(DRAW_ID, h, [(0.0, 0.0, 0.0, 0.0)] * fs_const_base + fconst)
    bd.fence(timeout=10.0)                                    # every resource in place, errors surfaced
    return len(idx)


def render_frame(bd, i, W, H, scene, instanced, vconst, speed, nidx, hold, batch=True, bones=False):
    a = scene.angle(i, speed)
    with (bd.batch() if batch else contextlib.nullcontext()):
        bd.frame_begin()
        if instanced:
            uni = vs_uniforms(scene, True, a, 0, vconst, bones)
            for u0 in range(0, len(uni), 80):                   # <= 80 vec4 per VS_UNIFORMS: one fits a BATCH datagram
                bd.vs_uniforms(u0, uni[u0:u0 + 80])             # (--bones: 6 + 4 N + constants vec4 per frame)
            bd.draw(0, nidx, 0, index_handle=H_IBO)
        else:
            for k in range(scene.n):
                bd.vs_uniforms(0, vs_uniforms(scene, False, a, k, vconst))
                bd.draw(0, 36, 0, index_handle=H_IBO)
        cc = tuple(fp.c16(c) for c in CLEAR)
        bd.flush(H_IMG[i & 1], fp.FL_CLEAR_COLOR | fp.FL_CMASK(0xF) | fp.FL_CLEAR_DEPTH | fp.FL_CLEAR_STENCIL, cc,
                 clear_depth=0xFFFFFF, scissor=(0, 0, W, H))
    return bd.present(H_IMG[i & 1], frame_id=i, hold=hold, wait=False)


def check_frame(bd, i, W, H, scene, ppm):
    """the frame's image from B's DDR against the host: every cube's front faces at the centre and 5 interior points,
    the clear colour at the corners; |difference| <= 2 per channel. The expected colour at a point is that of the
    NEAREST front face of any cube there (window z interpolated in the face's plane): with 32 cubes the cubes overlap,
    and a face's own colour is only expected where nothing is in front of it (otherwise other cubes, correctly in front,
    would count as mismatches)."""
    a = scene.angle(i, 1.0)
    data = bd.mem_read(B_IMG[i & 1], W * H * 4, board=1)
    def px(x, y): return int.from_bytes(data[(y * W + x) * 4:(y * W + x) * 4 + 4], 'little')
    def close(a_, b, tol=2): return all(abs(((a_ >> s) & 0xFF) - ((b >> s) & 0xFF)) <= tol for s in (24, 16, 8, 0))
    # every front face of every cube: window vertices (x, y, z) and colour
    front = []
    for k in range(scene.n):
        R = scene.rotation(a, k); mvp = scene.mvp(R, k)
        for face in FACES:
            win = []
            for p in face[2]:
                c = mat_vec(mvp, list(p) + [1.0])
                win.append(((c[0] / c[3]) * W / 2 + W / 2, (c[1] / c[3]) * H / 2 + H / 2, c[2] / c[3]))
            area = sum(win[j][0] * win[(j + 1) % 4][1] - win[(j + 1) % 4][0] * win[j][1] for j in range(4)) / 2
            if area > 0: front.append((k, face, win, area, rgba8(face_colour(R, face))))
    def tri_z(p0, p1, p2, x, y):                               # z of the plane through the triangle at (x, y), None outside
        d = (p1[0] - p0[0]) * (p2[1] - p0[1]) - (p2[0] - p0[0]) * (p1[1] - p0[1])
        if d == 0: return None
        l1 = ((x - p0[0]) * (p2[1] - p0[1]) - (p2[0] - p0[0]) * (y - p0[1])) / d
        l2 = ((p1[0] - p0[0]) * (y - p0[1]) - (x - p0[0]) * (p1[1] - p0[1])) / d
        l0 = 1 - l1 - l2
        if min(l0, l1, l2) < -1e-9: return None
        return l0 * p0[2] + l1 * p1[2] + l2 * p2[2]
    def nearest(x, y, own):                                    # the colour of the nearest front face at the pixel centre
        best = None
        for (k, face, win, area, col) in front:
            z = tri_z(win[0], win[1], win[2], x + 0.5, y + 0.5)
            if z is None: z = tri_z(win[0], win[2], win[3], x + 0.5, y + 0.5)
            if z is not None and (best is None or z < best[0]): best = (z, col, k, face[1])
        return best if best else (0.0, own, -1, None)
    bad = 0; faces = 0; occluded = 0
    for (k, face, win, area, exp_own) in front:
        if area <= 400: continue                               # small / edge-on faces are not sampled
        faces += 1
        cx, cy = sum(w[0] for w in win) / 4, sum(w[1] for w in win) / 4
        pts = [(cx, cy)] + [((cx + w[0]) / 2, (cy + w[1]) / 2) for w in win] + [((3 * cx + win[0][0]) / 4, (3 * cy + win[0][1]) / 4)]
        for sx, sy in pts:
            x, y = int(sx), int(sy)
            got = px(x, y)
            _, exp, kn, fn = nearest(x, y, exp_own)
            if kn != k: occluded += 1
            if not close(got, exp):
                bad += 1
                if bad <= 10: print(f"  check: cube {k} face {face[1]} pixel ({x}, {y}): {got:08x}, expected {exp:08x}"
                                    + (f" (cube {kn} face {fn} in front)" if kn != k else ""))
    exp_bg = rgba8(CLEAR[:3])
    for x, y in ((0, 0), (W - 1, 0), (0, H - 1), (W - 1, H - 1)):
        if px(x, y) != exp_bg:
            bad += 1; print(f"  check: corner ({x}, {y}): {px(x, y):08x}, expected the clear colour {exp_bg:08x}")
    print(f"check: frame {i}: {scene.n} cubes, {faces} front faces x 6 points ({occluded} points behind another cube) + 4 corners, {bad} mismatches")
    if ppm:
        path = os.path.join(GEN, 'cube_frame.ppm')
        with open(path, 'wb') as fh:
            fh.write(b"P6 %d %d 255\n" % (W, H))
            for y in range(H - 1, -1, -1):                     # GL origin: the last memory row is the top line
                row = data[y * W * 4:(y + 1) * W * 4]
                fh.write(bytes(b for x in range(W) for b in (row[4 * x + 3], row[4 * x + 2], row[4 * x + 1])))
        print(f"check: wrote {path}")
    return bad == 0


def err_text(e):
    seq, code, ty, det = e[:4]                                 # (seq, code, type, detail, sub-message index)
    if code == 9:
        return ("render frame aborted after %.1f ms: %s" % (det / 100e3, "the 3 s timeout (B took nothing)" if det >= RENDER_TIMEOUT_CYCLES
                                                              else "the board link went down (a retrain)"))
    return "%s (type %02X, detail %08X)" % (fp.ERR_NAMES.get(code, code), ty, det)


def main():
    a = sys.argv[1:]
    def opt(name, default=None, conv=str):
        return conv(a[a.index(name) + 1]) if name in a else default
    ip = opt('--ip', fp.BOARD_IP)
    frames = opt('--frames', None, int); secs = opt('--secs', None, float); speed = opt('--speed', 1.0, float)
    ncubes = opt('--cubes', 32, int); instanced = '--draws' not in a; hold = '--nohold' not in a; strict = '--strict' in a
    bones = '--bones' in a
    if bones and not instanced: raise SystemExit("--bones is one draw for every cube (not with --draws)")
    batch = '--nobatch' not in a
    forward = '--forward' in a                                 # compile with forward=True, as host/win/shc always does
    size = opt('--scale', 1.0, float)                          # cube size factor (0.05: a few pixels each = the per-frame floor)
    if not 1 <= ncubes <= MAX_CUBES: raise SystemExit(f"--cubes 1..{MAX_CUBES}")
    mode = opt('--mode', None, int)
    W, H = MODE_SIZE.get(mode if mode is not None else 1)
    scene = Scene(ncubes, W / H, size)
    if '--selftest' in a:                                      # the programs alone, no board: complete cores
        wv, wf, vconst, fconst, fs_cb = compile_programs(instanced, bones, ncubes, forward)
        if not selftest(scene, instanced, wv, wf, vconst, fconst, fs_cb, bones): sys.exit(1)
        return
    bd = fp.Board(ip)
    bd.hello()
    st = bd.status()
    noops = fp.stage_noops(st)                                 # the ops the board's cores lack are lowered by the compiler
    wv, wf, vconst, fconst, fs_cb = compile_programs(instanced, bones, ncubes, forward, noops)
    if '--check' in a and not selftest(scene, instanced, wv, wf, vconst, fconst, fs_cb, bones): sys.exit(1)
    bfl = fp.decode_b_flags(st['b_flags']); lfl = fp.decode_link_flags(st['link_flags'])
    if not (lfl['trained'] and lfl['b_alive']): raise fp.ProtoError("the board link is not up / B is not alive")
    if mode is not None: bd.set_mode(mode)
    else: mode = bfl['mode']
    W, H = MODE_SIZE[mode]; scene = Scene(ncubes, W / H, size)
    print(f"board A build {st['build']}, B mode {mode} ({W} x {H}); {ncubes} cube(s), "
          f"{'one draw (cubes_bones.vert: run-time indexed model matrices)' if bones else 'one draw (cubes.vert)' if instanced else 'a draw per cube (cube.vert)'}: "
          f"{len(wv)}-instruction vertex program, "
          f"{len(wf)}-instruction fragment program, {24 * ncubes} vertices, {12 * ncubes} triangles, {speed} deg per frame, "
          f"PRESENT {'hold (one frame per vsync)' if hold else 'without hold (as fast as rendered)'}, "
          f"{'BATCH datagrams' if batch else 'one datagram per command'}{fp.noops_note(noops)}")
    nidx = setup(bd, W, H, scene, instanced, wv, wf, vconst, fconst, fs_cb, bones)
    errs = bd.take_errors()
    for e in errs: print("  EV_ERROR seq %d: %s" % (e[0], err_text(e)))
    if errs: sys.exit(1)
    st0 = bd.status(); t0 = time.monotonic(); t_rep = t0; shown = 0; dropped = 0; rep_shown = 0; retrains = 0
    seq0 = bd.next_seq; hb0 = bd.n_hb_resend
    st_rep = st0; rep_frame = -1
    quads = collections.deque(maxlen=64); recs = collections.deque(maxlen=64)
    pending = collections.deque(); ok = True; i = -1
    try:
        while True:
            if frames is not None and i + 1 >= frames: break
            if secs is not None and time.monotonic() - t0 >= secs: break
            i += 1
            while len(pending) >= 2:
                fi, seq = pending.popleft()
                try:
                    _, _, _, drp = bd.wait_event(seq, timeout=5.0); shown += 1; dropped += drp
                except fp.ProtoError as e:                     # the PRESENT itself failed: reported with the errors below
                    print("  frame %d: %s" % (fi, e))
            seq = render_frame(bd, i, W, H, scene, instanced, vconst, speed, nidx, hold, batch, bones)
            pending.append((i, seq))
            if i == 0 and '--check' in a:
                _, _, _, drp = bd.wait_event(seq, timeout=10.0); shown += 1; dropped += drp; pending.clear()
                ok = check_frame(bd, 0, W, H, scene, '--ppm' in a)
            errs = bd.take_errors()
            fatal = False
            for e in errs:
                print("  EV_ERROR seq %d: %s" % (e[0], err_text(e)))
                if e[1] == 9 and e[3] < RENDER_TIMEOUT_CYCLES and not strict: retrains += 1
                else: fatal = True
            if fatal: ok = False; break
            now = time.monotonic()
            if now - t_rep >= 2.0:
                # the sample follows a FENCE: A's quad / record counters restart with every frame (the render's go, the
                # FRAME_BEGIN), so a STATUS read right after a frame's commands would see 0; after the fence they
                # hold the completed frame's values (one pipeline drain per sample: < 1 % of the frames)
                bd.fence(timeout=10.0)
                st = bd.status()
                quads.append(st['render_quads']); recs.append(st['geom_busy_tris'] & 0xFFFFF)
                print(f"{now - t0:6.1f} s  frame {i:6d}  shown {shown:6d} ({(shown - rep_shown) / (now - t_rep):6.1f} fps)  "
                      f"quads/frame {st['render_quads']:7d}  records {st['geom_busy_tris'] & 0xFFFFF:6d}  "
                      f"B underruns {st['b_underruns_moderr'] & 0xFFFF}  A errs {st['errs'] - st0['errs']}  link retrains {retrains}  "
                      f"resent {bd.n_resent} (heartbeat rule {bd.n_hb_resend - hb0})  A gap {st['gap'] - st0['gap']} dup {st['dup'] - st0['dup']}")
                if 'b_cyc_frame' in st:                             # STATUS layout 7: the split of B's last frame, A's share
                    nf = max(1, i - rep_frame); dt = now - t_rep
                    print("          " + fp.b_frame_split(st) +
                          "; A per frame: geometry %.2f ms, band walk %.2f ms, credit wait %.2f ms, render FIFO full %.2f ms" % tuple(
                              ((st[k] - st_rep[k]) & 0xFFFFFFFF) / fp.CLK_KHZ / nf for k in ('a_cyc_geom', 'a_cyc_render', 'a_cyc_rs_credit', 'a_cyc_rs_full')))
                    if st.get('layout', 0) >= 10: print("          " + fp.a_geom_split(st, st_rep, nf))
                    st_rep = st; rep_frame = i
                sys.stdout.flush(); t_rep = now; rep_shown = shown
    except KeyboardInterrupt:
        print("stopped")
    while pending:
        fi, seq = pending.popleft()
        try:
            _, _, _, drp = bd.wait_event(seq, timeout=5.0); shown += 1; dropped += drp
        except fp.ProtoError as e:
            print("  frame %d: %s" % (fi, e))
    bd.fence(timeout=30.0)
    st = bd.status(); el = time.monotonic() - t0
    rs = st['rs_pkts_inflight']; n = i + 1
    q = max(quads) if quads else st['render_quads']
    print(f"fb_cube: {ncubes} cube(s) {'one draw' if instanced else 'per-cube draws'}: {n} frames in {el:.1f} s = {n / el if el else 0:.1f} fps "
          f"({(n / el if el else 0) * q / 1e6:.2f} M quads/s at {q} quads/frame, {(n / el if el else 0) * 12 * ncubes / 1e3:.1f} k triangles/s), "
          f"shown {shown}, replaced before shown {dropped}; B vsyncs {st['b_frames'] - st0['b_frames']}, "
          f"B underruns {st['b_underruns_moderr'] & 0xFFFF}, A errs {st['errs'] - st0['errs']}, link retrains {retrains}, "
          f"render beats in flight {rs >> 16}; " + ("PASS" if ok and (rs >> 16) == 0 else "FAIL"))
    print(f"  host: {((bd.next_seq - seq0) & 0xFFFFFFFF) / n:.1f} datagrams per frame ({bd.n_batches} BATCH datagrams carrying "
          f"{bd.n_batched} commands), resent {bd.n_resent} (heartbeat rule {bd.n_hb_resend - hb0}), A gap {st['gap'] - st0['gap']} "
          f"dup {st['dup'] - st0['dup']}")
    if 'b_cyc_frame' in st: print("  last " + fp.b_frame_split(st))
    sys.exit(0 if ok and (rs >> 16) == 0 else 1)


if __name__ == '__main__': main()
