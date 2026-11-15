#!/usr/bin/env python3
"""fb_shader — a full-screen fragment shader on the two boards (A geometry -> B fragment core), shown on B's HDMI
output: the fragment-core performance measure (every pixel of the mode runs the program).

  python3 host/fb/fb_shader.py [--ip 192.168.1.10] [--mode M] [--secs S | --frames N] [--shader raymarch|spiral|heart]
                               [--steps N] [--hold] [--selftest] [--sim] [--check] [--forward]

Shaders (tests/fs_quad.vert passes the aspect-corrected screen position + the time as varying 0):
  raymarch   tests/fs_raymarch.frag: a sphere on a floor, sphere tracing with a hardware LOOP of --steps steps (default
             32) and a per-lane BREAK, central-difference normals, Lambert + ambient, gamma (≈ 240 instructions)
  spiral     tests/fs_spiral.frag: polar rainbow spiral, no loop: length, atan, 3 x COS, smoothstep (≈ 80 instructions)
  heart      tests/fs_heart.frag: a beating heart from Shadertoy, no loop: length, atan, 2 x pow, exp, sin, smoothstep;
             runs on the wall clock, the others on 0.03 s per frame
One draw of two triangles per frame covering the whole target; FLUSH clears the colour (the lazy fast clear), PRESENT
without the hold flag (frames run as fast as rendered; --hold: one frame per vsync). Every 2 s and at the end: fps,
quads per frame, B's per-frame cycle split (STATUS layout 7) and the cycles per quad = B's frame cycles / quads.
  --selftest  compile the programs and run a few pixels through tools/isa64.py's core simulator (no board); prints the
              simulator's micro-op count per warp = the core's issue cycles for 4 quads (the ideal cycles per quad / 4)
  --sim       the same before a board run
  --check     one frame read back from B's DDR, pixels against the same float maths, before the measurement
  --forward   compile with forward=True, as host/win/shc always does (the measure of that pass)
The frame split needs STATUS layout 7. Standard library + the repository's tools (glslc, isa64, float_ref).
"""
import collections, math, os, struct, sys, time
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(HERE, '..', '..', 'tools'))
import fbproto as fp
import glslc, isa64, float_ref as fr
from fb_image import MODE_SIZE

TESTS = os.path.join(HERE, '..', '..', 'tests')

# ---- resources (fb_cube's layout: A's host heap at 384 MB.., B's images at 0x3000_0000)
H_VBO, H_IBO, H_VSPROG = 0x0021, 0x0022, 0x0023                 # board A buffers
A_VBO, A_IBO, A_VSPROG = 0x1A00_0000, 0x1A40_0000, 0x1A50_0000
H_FSPROG, B_FSPROG = 0x8021, 0x1000_0000                        # board B program buffer (bit 15), 64 KB
H_IMG = (0xC030, 0xC031)
B_IMG = (0x3000_0000, 0x3040_0000)                              # 4 MB each: up to 1280 x 800 x 4
DRAW_ID = 1
RENDER_TIMEOUT_CYCLES = 300_000_000                             # fb_dispatch_a RENDER_TIMEOUT (3 s at 100 MHz)
CLEAR = (0.0, 0.0, 0.0, 1.0)
SHADERS = {'raymarch': 'fs_raymarch.frag', 'spiral': 'fs_spiral.frag', 'heart': 'fs_heart.frag'}


def compile_programs(shader, forward=False, noops=None):
    """noops: fbproto.stage_noops of the board (the units its cores lack), None = both cores complete"""
    noops = noops or {'vs': 0, 'fs': 0}
    rv = glslc.compile_stage(open(os.path.join(TESTS, 'fs_quad.vert')).read(), 'vs', isa64=True, forward=forward, noops=noops['vs'])
    rf = glslc.compile_stage(open(os.path.join(TESTS, SHADERS[shader])).read(), 'fs', isa64=True, forward=forward, noops=noops['fs'])
    wv, _ = isa64.assemble(rv['sasm'].splitlines(), vs=True)
    wf, _ = isa64.assemble(rf['sasm'].splitlines())
    assert rv['attribs'] == {'aPos': (0, 'vec2')}, rv['attribs']
    assert rv['uniforms'] == {'uParam': (0, 'vec4')} and rv['const_base'] == 1, rv['uniforms']
    assert rv['varyings'] == {'vUV': (0, 'vec4')} and rf['varyings'] == {'vUV': (0, 'vec4')}, (rv['varyings'], rf['varyings'])
    vconst = [tuple(float(c) for c in k) for k in rv['const_pool']]
    fconst = [tuple(float(c) for c in k) for k in rf['const_pool']]
    return wv, wf, vconst, fconst, rf['uniforms'], rf['const_base']


def fs_uniforms(funi, fconst, fs_cb, steps, res=(800, 600)):
    """the fragment program's uniform vectors: the user uniforms by slot (uSteps, iResolution), then the compiler's constants"""
    u = [(0.0, 0.0, 0.0, 0.0)] * fs_cb
    if 'uSteps' in funi: u[funi['uSteps'][0]] = (float(steps),) * 4
    if 'iResolution' in funi: u[funi['iResolution'][0]] = (float(res[0]), float(res[1]), 1.0, 0.0)
    return u + fconst


def words64_bytes(words):
    return b''.join(struct.pack("<II", w & 0xFFFFFFFF, w >> 32) for w in words)


def f24(x): return fp.fp24(float(x))
def real(b): return float(fr.to_real(b, fr.FP24))


# ------------------------------------------------------------------------------------------------ host reference (floats)
def ref_pixel(shader, x, y, tm, steps):
    """the shader's colour at screen position (x = ndc_x * aspect, y = ndc_y) in float arithmetic"""
    ux, uy = x * 0.5, y * 0.5
    if shader == 'heart':
        cl = lambda v, a, b: min(max(v, a), b)
        l = math.hypot(x, y)
        bcol = [c * (1.0 - 0.25 * l) for c in (1.0, 0.8, 0.7 - 0.07 * y)]
        tt = (tm % 1.5) / 1.5
        ss = tt ** 0.2 * 0.5 + 0.5
        ss = 1.0 + ss * 0.5 * math.sin(tt * 6.2831 * 3.0 + y * 0.5) * math.exp(-tt * 4.0)
        px, py = x * (0.5 + ss * 0.5), y * (1.5 - ss * 0.5) - 0.25
        h = abs(math.atan2(px, py) / 3.141593); r = math.hypot(px, py)
        d = (13.0 * h - 22.0 * h * h + 10.0 * h * h * h) / (6.0 - 5.0 * h)
        s = (0.75 + 0.75 * px) * (1.0 - 0.4 * r)
        s = 0.3 + 0.7 * s
        s *= 0.5 + 0.5 * (1.0 - cl(r / d, 0.0, 1.0) if d > 0.0 else 0.0) ** 0.1
        k = cl((d - r + 0.01) / 0.02, 0.0, 1.0); k = k * k * (3.0 - 2.0 * k)
        return [b + (c * s - b) * k for b, c in zip(bcol, (1.0, 0.4 * r, 0.3))]
    if shader == 'spiral':
        r = math.hypot(ux, uy); a = math.atan2(uy, ux) + tm
        col = [0.5 + 0.5 * math.cos(k + a * 3.0 + r * 8.0) for k in (0.0, 2.0, 4.0)]
        t = min(max((r - 0.52) / (0.50 - 0.52), 0.0), 1.0); v = t * t * (3 - 2 * t)
        return [c * v for c in col]
    def mp(p):
        s = math.sqrt(p[0] ** 2 + (p[1] - 0.25 * math.sin(tm)) ** 2 + p[2] ** 2) - 1.0
        return min(s, p[1] + 1.0)
    ro = (0.0, 0.6, 3.0); l = math.sqrt(ux * ux + uy * uy + 1.4 * 1.4); rd = (ux / l, uy / l, -1.4 / l)
    t = 0.0; hit = False
    for _ in range(steps):
        p = [ro[k] + rd[k] * t for k in range(3)]
        d = mp(p)
        if d < 0.002: hit = True; break
        t += d
        if t > 12.0: break
    col = [0.55 - 0.3 * uy, 0.72 - 0.3 * uy, 0.95 - 0.3 * uy]
    if hit:
        p = [ro[k] + rd[k] * t for k in range(3)]; e = 0.008
        n = [mp([p[0] + e, p[1], p[2]]) - mp([p[0] - e, p[1], p[2]]), mp([p[0], p[1] + e, p[2]]) - mp([p[0], p[1] - e, p[2]]),
             mp([p[0], p[1], p[2] + e]) - mp([p[0], p[1], p[2] - e])]
        ln = math.sqrt(sum(c * c for c in n)); n = [c / ln for c in n]
        lig = (0.7, 0.8, 0.5); ll = math.sqrt(sum(c * c for c in lig)); lig = [c / ll for c in lig]
        dif = max(sum(a * b for a, b in zip(n, lig)), 0.0)
        base = (0.35, 0.55, 0.35) if p[1] < -0.9 else (0.95, 0.45, 0.30)
        col = [b * (dif + 0.35) for b in base]
    return [max(c, 0.0) ** 0.4545 for c in col]


def selftest(shader, wf, fconst, funi, fs_cb, steps, aspect=4.0 / 3.0):
    """4 quads (16 pixels) through the core simulator: colours against the host's float maths (loose: the hardware's
    SIN / COS / RSQRT / LOG2 / EXP2 are FP24 approximations and the march's break point may differ by a step), the
    simulator's micro-op count = the core's issue cycles for the warp"""
    W, H = 800, 600
    params = [[f24(c) for c in v] for v in fs_uniforms(funi, fconst, fs_cb, steps, (W, H))]
    params += [[0] * 4] * (64 - len(params))
    tm = 0.7; pts = []
    for q, (cx, cy) in enumerate(((0.05, 0.03), (0.3, -0.5), (-0.6, 0.2), (0.9, 0.8))):   # off the x = 0 line: atan's seam
        for p in range(4): pts.append((cx + 0.02 * (p & 1), cy - 0.02 * (p >> 1)))
    varin = [[[f24(x * aspect), f24(y), f24(tm), f24(1.0)]] + [[0] * 4] * 7 + [[f24((x + 1.0) * 0.5 * W), f24((y + 1.0) * 0.5 * H), 0, f24(1.0)]] + [[0] * 4] * 7
             for x, y in pts]    # slot 8 = gl_FragCoord
    res = isa64.simulate_fs2(wf, isa64.FSInputs(params, varin, active=0xFFFF, interp=False))
    bad = 0; worst = 0.0
    for l, (x, y) in enumerate(pts):
        got = [real(c) for c in res.outc[l][:3]]; exp = ref_pixel(shader, x * aspect, y, tm, steps)
        d = max(abs(g - e) for g, e in zip(got, exp)); worst = max(worst, d)
        if d > 0.05 or (res.killed >> l) & 1:
            bad += 1
            if bad <= 6: print(f"  selftest: pixel {l} ({x * aspect:.3f}, {y:.3f}): core {[round(g, 3) for g in got]}, host {[round(e, 3) for e in exp]}"
                               f"{' killed' if (res.killed >> l) & 1 else ''}")
    print(f"selftest {shader} steps {steps}: {len(wf)} instructions; one warp of 16 pixels: {res.uops} micro-ops "
          f"(ideal {res.uops / 4:.0f} cycles per quad), core error {int(res.err)}, {bad} pixels off by > 0.05 (worst {worst:.3f})")
    return bad == 0 and not res.err


# ------------------------------------------------------------------------------------------------ the board
def setup(bd, W, H, wv, wf, fconst, funi, fs_cb, steps):
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
    uni = fs_uniforms(funi, fconst, fs_cb, steps, (W, H))
    h = [0] * 23
    h[0] = 0xF << 16                                           # no depth test, colour mask RGBA, no blend
    h[11] = len(wf) << 16 | H_FSPROG; h[12] = 0; h[13] = len(uni)
    bd.draw_state(DRAW_ID, h, uni)
    bd.fence(timeout=10.0)
    return len(idx)


def render_frame(bd, i, W, H, aspect, tm, vconst, nidx, hold):
    bd.frame_begin()
    bd.vs_uniforms(0, [(aspect, tm, 0.0, 0.0)] + vconst)
    bd.draw(0, nidx, 0, index_handle=H_IBO)
    cc = tuple(fp.c16(c) for c in CLEAR)
    bd.flush(H_IMG[i & 1], fp.FL_CLEAR_COLOR | fp.FL_CMASK(0xF), cc, scissor=(0, 0, W, H))
    return bd.present(H_IMG[i & 1], frame_id=i, hold=hold, wait=False)


def check_frame(bd, W, H, aspect, shader, tm, steps):
    """the frame just shown, from B's DDR: ~100 pixels against the host's float maths (ref_pixel), rows bottom-up as GL's
    y and scan-out's origin 1 have them, so an upside-down picture fails. Off by > 3 % = wrong; the heart's soft edge
    moves with the FP24 approximations, so pixels within 0.03 of it are skipped"""
    data = bd.mem_read(B_IMG[0], W * H * 4, board=1)
    n = bad = 0; worst = 0.0
    for k in range(400):
        px = (k * 97 + 13) % W; py = (k * 61 + 7) % H
        x = ((px + 0.5) / W * 2.0 - 1.0) * aspect; y = (py + 0.5) / H * 2.0 - 1.0
        exp = ref_pixel(shader, x, y, tm, steps)
        if shader == 'heart' and any(abs(a - b) > 0.002 for a, b in zip(exp, ref_pixel(shader, x + 0.03, y, tm, steps))) and \
           max(abs(a - b) for a, b in zip(exp, ref_pixel(shader, x + 0.03, y, tm, steps))) > 0.06: continue
        o = (py * W + px) * 4
        got = (data[o + 3] / 255.0, data[o + 2] / 255.0, data[o + 1] / 255.0)
        d = max(abs(g - min(max(e, 0.0), 1.0)) for g, e in zip(got, exp)); worst = max(worst, d); n += 1
        if d > 0.03:
            bad += 1
            if bad <= 5: print(f"  check: pixel ({px}, {py}) = {[round(g, 3) for g in got]}, expected {[round(e, 3) for e in exp]}")
        if n >= 100: break
    print(f"  check {shader}: {n} pixels compared, {bad} wrong, worst {worst:.3f}")
    return n > 0 and bad == 0


def err_text(e):
    seq, code, ty, det = e[:4]                                 # (seq, code, type, detail, sub-message index)
    if code == 9:
        return ("render frame aborted after %.1f ms: %s" % (det / 100e3, "the 3 s timeout (B took nothing / the frame is longer than 3 s)"
                                                              if det >= RENDER_TIMEOUT_CYCLES else "the board link went down (a retrain)"))
    return "%s (type %02X, detail %08X)" % (fp.ERR_NAMES.get(code, code), ty, det)


def main():
    a = sys.argv[1:]
    def opt(name, default=None, conv=str):
        return conv(a[a.index(name) + 1]) if name in a else default
    ip = opt('--ip', fp.BOARD_IP)
    frames = opt('--frames', None, int); secs = opt('--secs', 5.0 if '--frames' not in a else None, float)
    shader = opt('--shader', 'raymarch'); steps = opt('--steps', 32, int); hold = '--hold' in a
    if shader not in SHADERS: raise SystemExit("--shader " + " | ".join(SHADERS))
    mode = opt('--mode', None, int)
    if '--selftest' in a:                                      # the programs alone, no board: complete cores
        wv, wf, vconst, fconst, funi, fs_cb = compile_programs(shader, '--forward' in a)
        if not selftest(shader, wf, fconst, funi, fs_cb, steps): sys.exit(1)
        return
    bd = fp.Board(ip)
    bd.hello()
    st = bd.status()
    noops = fp.stage_noops(st)                                 # the ops the board's cores lack are lowered by the compiler
    wv, wf, vconst, fconst, funi, fs_cb = compile_programs(shader, '--forward' in a, noops)
    if '--sim' in a and not selftest(shader, wf, fconst, funi, fs_cb, steps): sys.exit(1)
    bfl = fp.decode_b_flags(st['b_flags']); lfl = fp.decode_link_flags(st['link_flags'])
    if not (lfl['trained'] and lfl['b_alive']): raise fp.ProtoError("the board link is not up / B is not alive")
    if mode is not None: bd.set_mode(mode)
    else: mode = bfl['mode']
    W, H = MODE_SIZE[mode]; aspect = W / H
    nquads = (W // 2) * (H // 2)
    print(f"board A build {st['build']}, B mode {mode} ({W} x {H}, {nquads} quads); {shader}{' steps %d' % steps if shader == 'raymarch' else ''}: "
          f"{len(wv)}-instruction vertex program, {len(wf)}-instruction fragment program, PRESENT "
          f"{'hold (one frame per vsync)' if hold else 'without hold (as fast as rendered)'}{fp.noops_note(noops)}")
    nidx = setup(bd, W, H, wv, wf, fconst, funi, fs_cb, steps)
    errs = bd.take_errors()
    for e in errs: print("  EV_ERROR seq %d: %s" % (e[0], err_text(e)))
    if errs: sys.exit(1)
    checked = True
    if '--check' in a:
        bd.wait_event(render_frame(bd, 0, W, H, aspect, 0.2, vconst, nidx, False), timeout=20.0)
        checked = check_frame(bd, W, H, aspect, shader, 0.2, steps)
    st0 = bd.status(); t0 = time.monotonic(); t_rep = t0; shown = 0; dropped = 0; rep_shown = 0; retrains = 0
    pending = collections.deque(); ok = checked; i = -1; fsplits = []
    try:
        while True:
            if frames is not None and i + 1 >= frames: break
            if secs is not None and time.monotonic() - t0 >= secs: break
            i += 1
            while len(pending) >= 2:
                fi, seq = pending.popleft()
                try:
                    _, _, _, drp = bd.wait_event(seq, timeout=20.0); shown += 1; dropped += drp
                except fp.ProtoError as e:
                    print("  frame %d: %s" % (fi, e))
            seq = render_frame(bd, i, W, H, aspect, time.monotonic() - t0 if shader == 'heart' else 0.03 * i, vconst, nidx, hold)
            pending.append((i, seq))
            errs = bd.take_errors(); fatal = False
            for e in errs:
                print("  EV_ERROR seq %d: %s" % (e[0], err_text(e)))
                if e[1] == 9 and e[3] < RENDER_TIMEOUT_CYCLES: retrains += 1
                else: fatal = True
            if fatal: ok = False; break
            now = time.monotonic()
            if now - t_rep >= 2.0:
                bd.fence(timeout=30.0)
                st = bd.status()
                fps = (shown - rep_shown) / (now - t_rep)
                print(f"{now - t0:6.1f} s  frame {i:6d}  shown {shown:6d} ({fps:6.2f} fps)  quads/frame {st['render_quads']:7d}  "
                      f"A errs {st['errs'] - st0['errs']}  link retrains {retrains}  resent {bd.n_resent}")
                if 'b_cyc_frame' in st:
                    q = max(1, st['render_quads'])
                    fsplits.append((st['b_cyc_frame'], st['b_cyc_fs'], q))
                    print("          " + fp.b_frame_split(st) + f"; {st['b_cyc_frame'] / q:.1f} cycles per quad "
                          f"({st['b_cyc_fs'] / q:.1f} FS busy per quad)")
                sys.stdout.flush(); t_rep = now; rep_shown = shown
    except KeyboardInterrupt:
        print("stopped")
    while pending:
        fi, seq = pending.popleft()
        try:
            _, _, _, drp = bd.wait_event(seq, timeout=20.0); shown += 1; dropped += drp
        except fp.ProtoError as e:
            print("  frame %d: %s" % (fi, e))
    bd.fence(timeout=30.0)
    st = bd.status(); el = time.monotonic() - t0
    rs = st['rs_pkts_inflight']; n = i + 1; q = max(1, st['render_quads'])
    fps = n / el if el else 0
    print(f"fb_shader {shader}{' steps %d' % steps if shader == 'raymarch' else ''} {W}x{H}: {n} frames in {el:.1f} s = {fps:.2f} fps "
          f"({fps * q / 1e6:.3f} M quads/s at {q} quads/frame), shown {shown}, replaced before shown {dropped}; "
          f"A errs {st['errs'] - st0['errs']}, link retrains {retrains}, render beats in flight {rs >> 16}; " +
          ("PASS" if ok and (rs >> 16) == 0 else "FAIL"))
    if 'b_cyc_frame' in st:
        print(f"  last {fp.b_frame_split(st)}; {st['b_cyc_frame'] / q:.1f} cycles per quad, FS busy {st['b_cyc_fs'] / q:.1f} per quad, "
              f"{len(wf)} instructions")
        print(f"RESULT {shader} steps={steps} mode={mode} {W}x{H} fps={fps:.2f} cyc_per_quad={st['b_cyc_frame'] / q:.1f} "
              f"fs_busy={100.0 * st['b_cyc_fs'] / max(1, st['b_cyc_frame']):.1f}% ropstall={100.0 * st['b_cyc_ropstall'] / max(1, st['b_cyc_frame']):.1f}% "
              f"starved={100.0 * st['b_cyc_starved'] / max(1, st['b_cyc_frame']):.1f}% instr={len(wf)} quads={q}")
    sys.exit(0 if ok and (rs >> 16) == 0 else 1)


if __name__ == '__main__': main()
