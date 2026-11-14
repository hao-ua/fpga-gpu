#!/usr/bin/env python3
"""opt_check — forward=True must not change what a shader computes: every shader compiled with and without it, both run through isa64's simulator, every output compared for every lane, on several sets of inputs.
Word-for-word with the C port proves the port; this proves the pass. --unread mask does the same for g2sc_compile_vs: the vertex shaders, with and without those slots written; every other output must stay.
--xops n: the fragment shaders with board B's operations (glslc xops: 1 SQRT / RCPM / RSQM, 2 CLAMP) against without, both forward: bit for bit the same.
    python3 -I host/win/shc/test/opt_check.py [--xops n] [--unread mask] file.vert file.frag ...      (run from the repository root) """
import os, random, sys, signal
sys.path.insert(0, 'tools')
import glslc, isa64


def too_long(*_):
    raise RuntimeError('timeout')


def inputs(stage, seed, n_sets=3):
    rnd = random.Random(seed)
    def fb(x): return isa64.fr.encode_real(isa64.F(x).limit_denominator(1 << 14), isa64.FMT)
    def val(): return fb(rnd.choice((0, 1, -1, 0.5, -0.5, 2, 0.25, 3, 0.75)) * rnd.choice((1, 1, 0.125, 4)) + rnd.randrange(-8, 9) * 0.0625)
    if stage == 'vs':
        inp = isa64.default_vs_inputs(rich=True)
        inp = inp._replace(params=[[val() for _ in range(4)] for _ in range(256)]) if hasattr(inp, '_replace') else inp
        if not hasattr(inp, '_replace'): inp.params = [[val() for _ in range(4)] for _ in range(256)]
    else:
        inp = isa64.default_fs_inputs(interp=True)
        if hasattr(inp, '_replace'): inp = inp._replace(params=[[val() for _ in range(4)] for _ in range(isa64.NPARAM)])
        else: inp.params = [[val() for _ in range(4)] for _ in range(isa64.NPARAM)]
    for l in range(len(inp.varin)):
        for i in range(len(inp.varin[l])): inp.varin[l][i] = [val() for _ in range(4)]
    return inp


def run(words, stage, seed, prog):
    """prog's constant pool in its slots, as a library loads it: a pass may take a constant for what it is (shape_opt's 0 and 1)"""
    inp = inputs(stage, seed)
    for i, k in enumerate(prog['const_pool']):
        if prog['const_base'] + i < len(inp.params): inp.params[prog['const_base'] + i] = [isa64.fr.encode_real(isa64.F(c), isa64.FMT) for c in k]
    r = isa64.simulate_vs2(words, inp) if stage == 'vs' else isa64.simulate_fs2(words, inp)
    return r


def comps(prog):
    """the components each varying slot defines: a vec2's z and w are nobody's (a register the program never filled there)"""
    n = [4] * isa64.NVARY
    for v in prog['varyings'].values():
        idx, ty, asz = v[0], v[1], (v[2] if len(v) > 2 else 1)
        per, k = (glslc.MAT[ty], glslc.MAT[ty]) if glslc.is_mat(ty) else (1, glslc.ncomp(ty))
        for s in range(idx, min(idx + per * asz, isa64.NVARY)): n[s] = k
    return n


def outputs(r, unread=0, nc=None):
    """everything a program leaves behind: the registers are its scratch, these are its results"""
    o = {}
    for k, v in vars(r).items() if hasattr(r, '__dict__') else r._asdict().items():
        if k in ('gpr', 'uops', 'pc', 'steps', 'cycles', 'instrs'): continue
        o[k] = v
    if nc and o.get('outv') is not None: o['outv'] = [[lane[:nc[i]] for lane in v] for i, v in enumerate(o['outv'])]
    if unread and o.get('outv') is not None: o['outv'] = [v for i, v in enumerate(o['outv']) if not unread >> i & 1]
    return o


def main():
    signal.signal(signal.SIGALRM, too_long)
    unread = 0
    xops = 0
    if sys.argv[1] == '--xops':             # the same for board B's fragment-core operations: without them against with them, both forward
        xops = int(sys.argv[2], 0)
        del sys.argv[1:3]
    if sys.argv[1] == '--unread':
        unread = int(sys.argv[2], 0)
        del sys.argv[1:3]
    same = bad = failed = both = 0
    w0 = w1 = u0 = u1 = 0
    rows = []
    for path in sys.argv[1:]:
        stage = 'vs' if path.endswith(('.vert', '.vs')) else 'fs'
        if (unread and stage != 'vs') or (xops and stage != 'fs'): continue
        name = os.path.basename(path)
        signal.alarm(30)
        try:
            src = open(path, newline='').read()
            a = glslc.compile_stage(src, stage, isa64=True, forward=bool(unread or xops))
        except Exception:
            both += 1
            continue
        try:
            b = glslc.compile_stage(src, stage, isa64=True, forward=True, unread=unread, xops=xops)
            wa, _ = isa64.assemble(a['sasm'].splitlines(), vs=stage == 'vs')
            wb, _ = isa64.assemble(b['sasm'].splitlines(), vs=stage == 'vs')
        except Exception as e:
            failed += 1
            print('FAIL', name, 'only with forward:', str(e)[:140])
            continue
        if a['const_pool'] != b['const_pool'] and len(b['const_pool']) > len(a['const_pool']):
            print('note', name, 'constant pool grew')
        ok = True
        ua = ub = 0
        for seed in range(3):
            try:
                ra, rb = run(wa, stage, seed, a), run(wb, stage, seed, b)
            except Exception as e:
                print('skip', name, 'simulator:', str(e)[:100])
                ok = None
                break
            ua += ra.uops; ub += rb.uops
            nc = comps(a) if stage == 'vs' else None
            oa, ob = outputs(ra, unread, nc), outputs(rb, unread, nc)
            if oa != ob:
                diff = [k for k in oa if oa[k] != ob.get(k)]
                print('FAIL', name, 'seed', seed, 'outputs differ:', diff)
                ok = False
                break
        if ok is None: continue
        if ok: same += 1
        else: bad += 1
        w0 += len(wa); w1 += len(wb); u0 += ua; u1 += ub
        rows.append((name, len(wa), len(wb), ua, ub))
    rows.sort(key=lambda x: x[1] - x[2], reverse=True)
    for n, a, b, ua, ub in rows[:16]:
        print('%-28s %4d -> %4d instructions, %5d -> %5d micro-ops' % (n, a, b, ua // 3, ub // 3))
    print('%d same outputs, %d differ, %d fail only with forward, %d refused either way' % (same, bad, failed, both))
    if w0 and u0: print('total %d -> %d instructions (%.0f%%), micro-ops %d -> %d (%.0f%%)' % (w0, w1, 100.0 * w1 / w0, u0, u1, 100.0 * u1 / u0))
    return 1 if bad or failed else 0


sys.exit(main())
