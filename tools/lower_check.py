#!/usr/bin/env python3
"""lower_check — the compilers' replacements for the units a shader core may lack (glslc.compile_stage noops: 1 the integer
divider, 2 SIN / COS; STATUS flags bits 30 / 31), run on the ISA simulator (isa64, the RTL's RCP).

  1. sin / cos: a vertex program over many arguments (exact points, tiny, [-pi, pi], wider ranges, huge): |result| <= 1,
     sin(0) = 0, cos(0) = 1 exactly, the largest error per range against math.sin / cos, beside the SIN / COS unit's.
  2. int /: a vertex program dividing 24-bit two's-complement ints (built with IMUL / IADD), the quotient split exactly
     into two floats: the lowered program must give what the program with IDIV gives, for random and edge operands.
  3. tests/lower_*.vert / .frag on the default bench inputs, compiled with noops = 3 and without: no IDIV / SIN / COS left,
     the integer outputs identical, the others within the sin / cos error. --rtl also writes tools/gen/<name>_vlow.sasm (vertex)
     / <name>_flow.sasm (fragment) and the tb_vs2 / tb_fs2 golden set of the lowered program (isa64.main_vs2 / main_fs2, with
     its constant pool in the parameters), to run
         powershell -File tools/sim_gpu.ps1 -Test tb_vs2 -Plusargs PROG_lower_idiv_vlow,WARPS_4,TOL_8,OUTONLY
         powershell -File tools/sim_gpu.ps1 -Test tb_fs2 -Plusargs PROG_lower_sincos_flow,WARPS_4,TOL_8,OUTONLY

    python3 tools/lower_check.py [--n N] [--rtl]          (N: random integer divisions, default 4000)
"""
import os, sys, math, random
from fractions import Fraction as F
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import glslc, isa64, float_ref as fr, glslc64_check as gc

FMT = fr.FP24
TESTS = os.path.join(os.path.dirname(HERE), 'tests')
U = isa64.Units(exact_rcp=True)
def fb(x): return fr.encode_real(F(x), FMT)
def val(b):
    r = fr.to_real(b, FMT)
    return float(r) if r is not None else float('nan')

SINCOS_VS = """attribute vec4 a;
varying vec4 vs;
varying vec4 vc;
void main() { vs = sin(a); vc = cos(a); gl_Position = a; }"""
IDIV_VS = """attribute vec4 a;
varying vec4 o;
void main() {
    int k = int(a.x) * 4096 + int(a.y);
    int n = int(a.z) * 4096 + int(a.w);
    int q = k / n;
    int h = q / 4096;
    o = vec4(float(h), float(q - h * 4096), float(k - q * n), 0.0);
    gl_Position = a;
}"""


def compiled(src, stage, noops):
    r = glslc.compile_stage(src, stage, isa64=True, forward=True, noops=noops)
    words, _ = isa64.assemble(r['sasm'].splitlines(), vs=stage == 'vs')
    return r, words


def count(r, *ops):
    return sum(1 for l in r['sasm'].splitlines() if l.split() and l.split()[0].split('.')[0] in ops)


def run_vs(r, words, attrs):
    """attrs: {attribute name: [vec4 of floats or bits per lane]} for up to 16 vertices -> FSResult"""
    inp = isa64.default_vs_inputs()
    inp.params = gc.params_for(r, random.Random(11), 256, True)
    n = len(next(iter(attrs.values())))
    for name, lanes in attrs.items():
        slot = r['attribs'][name][0]
        for l, v in enumerate(lanes): inp.varin[l][slot] = list(v)
    inp.active = (1 << n) - 1
    return isa64.simulate_vs2(words, inp, U)


def sincos_check():
    r0, w0 = compiled(SINCOS_VS, 'vs', 0)
    r2, w2 = compiled(SINCOS_VS, 'vs', 2)
    assert count(r2, 'SIN', 'COS') == 0 and count(r0, 'SIN', 'COS') == 2
    so, co = r2['varyings']['vs'][0], r2['varyings']['vc'][0]
    rng = random.Random(3)
    ranges = [("exact 0, +-pi/2, +-pi", [0.0, -0.0, math.pi / 2, -math.pi / 2, math.pi, -math.pi, 2 * math.pi]),
              ("tiny 1e-7 .. 1e-2", [s * 10 ** rng.uniform(-7, -2) for s in (1, -1) for _ in range(160)]),
              ("[-pi, pi]", [rng.uniform(-math.pi, math.pi) for _ in range(640)]),
              ("[-100, 100]", [rng.uniform(-100, 100) for _ in range(320)]),
              ("[-1e4, 1e4]", [rng.uniform(-1e4, 1e4) for _ in range(320)]),
              ("huge 1e5 .. 1e18", [s * 10 ** rng.uniform(5, 18) for s in (1, -1) for _ in range(64)])]
    ok = True
    print("sin / cos of a vec4 each: %d instructions lowered, %d with the unit; largest |error| per range:" % (len(w2), len(w0)))
    for name, xs in ranges:
        xb = [fb(x) for x in xs]
        worst = [0.0] * 4                    # lowered sin, cos; unit sin, cos
        over = 0
        for i in range(0, len(xb), 64):
            chunk = xb[i:i + 64]
            while len(chunk) % 4: chunk.append(chunk[-1])
            lanes = [chunk[k:k + 4] for k in range(0, len(chunk), 4)]
            res2 = run_vs(r2, w2, {'a': lanes}); res0 = run_vs(r0, w0, {'a': lanes})
            for l, v in enumerate(lanes):
                for c in range(4):
                    x = float(fr.to_real(v[c], FMT))
                    t = (math.sin(x), math.cos(x))
                    for j, (res, slot) in enumerate(((res2, so), (res2, co), (res0, so), (res0, co))):
                        y = val(res.outv[slot][l][c])
                        if j < 2 and not abs(y) <= 1.0: over += 1
                        if abs(x) < 1e5: worst[j] = max(worst[j], abs(y - t[j % 2]))
        exact = ""
        if name.startswith("exact"):
            res2 = run_vs(r2, w2, {'a': [[fb(0.0)] * 4]})
            s0, c0 = res2.outv[so][0][0], res2.outv[co][0][0]
            exact = " sin(0) = %r, cos(0) = %r" % (val(s0), val(c0))
            if s0 & 0x7FFFFF or c0 != fb(1.0): ok = False; exact += " WRONG"
        if over: ok = False
        if name.startswith("huge"):
            print("  %-22s |r| > 1: %d (the phase of such arguments is noise in FP24)" % (name, over))
        else:
            print("  %-22s lowered sin %.2e cos %.2e | unit sin %.2e cos %.2e | |r| > 1: %d%s"
                  % (name, worst[0], worst[1], worst[2], worst[3], over, exact))
    return ok


def idiv_check(n):
    r0, w0 = compiled(IDIV_VS, 'vs', 0)
    r1, w1 = compiled(IDIV_VS, 'vs', 1)
    assert count(r1, 'IDIV') == 0 and count(r0, 'IDIV') == 2
    o = r1['varyings']['o'][0]
    rng = random.Random(5)
    edge = sorted({s * ((1 << k) + d) for k in range(24) for d in (-1, 0, 1) for s in (1, -1)} | {0, 3, 7, 10, 1000, 65535})
    edge = [e for e in edge if -(1 << 23) <= e < (1 << 23)]
    cases = [(a, b) for a in edge for b in edge if b != 0]
    rng.shuffle(cases); cases = cases[:n // 2]
    for _ in range(n // 2):
        kind = rng.random()
        if kind < 0.5: a, b = rng.randrange(-(1 << 23), 1 << 23), rng.randrange(-(1 << 23), 1 << 23)
        elif kind < 0.8: a, b = rng.randrange(-(1 << 23), 1 << 23), rng.choice((1, -1)) * rng.randrange(1, 1 << rng.randrange(1, 12))
        else: a, b = rng.randrange(-(1 << 16), 1 << 16), rng.randrange(-(1 << 16), 1 << 16)
        if b: cases.append((a, b))
    def split(v): h, l = v >> 12, v & 4095; return [fb(h), fb(l)]   # v = h * 4096 + l, l in [0, 4096)
    def wrap(v): return ((v + (1 << 23)) & ((1 << 24) - 1)) - (1 << 23)   # 24-bit two's complement (-2^23 / -1 wraps)
    bad = 0
    for i in range(0, len(cases), 16):
        chunk = cases[i:i + 16]
        lanes = [split(a) + split(b) for a, b in chunk]
        res1 = run_vs(r1, w1, {'a': lanes}); res0 = run_vs(r0, w0, {'a': lanes})
        for l, (a, b) in enumerate(chunk):
            q = wrap(abs(a) // abs(b) * (1 if (a < 0) == (b < 0) else -1))
            h = int(q / 4096)
            want = [fb(h), fb(q - h * 4096), fb(wrap(a - q * b))]
            got1 = res1.outv[o][l][:3]; got0 = res0.outv[o][l][:3]
            if got1 != got0 or got0 != want:
                bad += 1
                if bad <= 8: print("  %d / %d: lowered %s, IDIV %s, exact %s" % (a, b, [val(x) for x in got1], [val(x) for x in got0], [val(x) for x in want]))
    zero = run_vs(r1, w1, {'a': [split(a) + split(0) for a in (0, 1, -1, 12345, -(1 << 23), (1 << 23) - 1)]})
    fin = all(not math.isnan(val(zero.outv[o][l][0])) for l in range(6)) and not zero.err
    print("int / (two per vertex): %d instructions lowered, %d with IDIV; %d divisions (edges and random 24-bit), %d differ "
          "from IDIV or the exact quotient; x / 0 finite, no stack error: %s" % (len(w1), len(w0), len(cases), bad, fin))
    return bad == 0 and fin


def programs_check(rtl):
    ok = True
    for fn in sorted(os.listdir(TESTS)):
        if not fn.startswith('lower_') or os.path.splitext(fn)[1] not in ('.vert', '.frag'): continue
        stage = 'vs' if fn.endswith('.vert') else 'fs'
        src = open(os.path.join(TESTS, fn)).read()
        r0 = glslc.compile_stage(src, stage, isa64=True, forward=True)
        r3 = glslc.compile_stage(src, stage, isa64=True, forward=True, noops=3)
        left = count(r3, 'IDIV', 'SIN', 'COS')
        name = os.path.splitext(fn)[0] + ('_vlow' if stage == 'vs' else '_flow')
        path = os.path.join(isa64.GEN, name + '.sasm')
        with open(path, 'w') as f: f.write(r3['sasm'] + '\n')
        # the bench's default inputs with each program's own constant pool (and the same user uniforms) in the parameters
        nslots = 256 if stage == 'vs' else 64
        p3 = gc.params_for(r3, random.Random(11), nslots, stage == 'vs'); p0 = gc.params_for(r0, random.Random(11), nslots, stage == 'vs')
        if stage == 'vs':
            w3, _ = isa64.assemble(r3['sasm'].splitlines(), vs=True); w0, _ = isa64.assemble(r0['sasm'].splitlines(), vs=True)
            inp3 = isa64.default_vs_inputs(); inp3.params = p3; inp0 = isa64.default_vs_inputs(); inp0.params = p0
            res3 = isa64.simulate_vs2(w3, inp3, U); res0 = isa64.simulate_vs2(w0, inp0, U)
            outs = [(res3.outv[i][l][c], res0.outv[i][l][c]) for i in range(16) for l in range(16) for c in range(4)]
        else:
            w3, _ = isa64.assemble(r3['sasm'].splitlines()); w0, _ = isa64.assemble(r0['sasm'].splitlines())
            inp3 = isa64.default_fs_inputs(True); inp3.params = p3; inp0 = isa64.default_fs_inputs(True); inp0.params = p0
            res3 = isa64.simulate_fs2(w3, inp3, U); res0 = isa64.simulate_fs2(w0, inp0, U)
            outs = [(res3.outc[l][c], res0.outc[l][c]) for l in range(16) for c in range(4)]
        diff = max(abs(val(a) - val(b)) for a, b in outs)
        same = sum(1 for a, b in outs if a == b)
        idiv_only = 'sin' not in src and 'cos' not in src
        good = left == 0 and not res3.err and (diff == 0 if idiv_only else diff < 1e-3)
        ok &= good
        print("%s %-18s %4d -> %4d instructions, IDIV / SIN / COS left %d, outputs equal %d / %d, largest difference %.2e"
              % ('ok  ' if good else 'FAIL', fn, len(w0), len(w3), left, same, len(outs), diff))
        if rtl:                              # the golden set with the lowered program's constants in the parameters
            orig = isa64.default_vs_inputs if stage == 'vs' else isa64.default_fs_inputs
            def with_params(*a, **k):
                inp = orig(*a, **k); inp.params = p3; return inp
            if stage == 'vs': isa64.default_vs_inputs = with_params; isa64.main_vs2(path, exact_rcp=True); isa64.default_vs_inputs = orig
            else: isa64.default_fs_inputs = with_params; isa64.main_fs2(path, exact_rcp=True); isa64.default_fs_inputs = orig
    return ok


def main():
    a = sys.argv[1:]
    n = int(a[a.index('--n') + 1]) if '--n' in a else 4000
    ok = sincos_check()
    ok &= idiv_check(n)
    ok &= programs_check('--rtl' in a)
    print("LOWER_CHECK_PASS" if ok else "LOWER_CHECK_FAIL")
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
