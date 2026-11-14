#!/usr/bin/env python3
"""pack_check — GLSL ES Appendix A.7: a program past one row a variable. Each program below compiles with a layout from the same placement as g2sc_pack_vars, runs through the vertex and the fragment simulator with the
uniforms stored where the layout says, and every varying that crosses is compared with its value.
    python3 -I host/win/shc/test/pack_check.py [--dump dir]      (run from the repository root; dump: the shaders and their .pack files, for compare.ps1) """
import os, sys
sys.path.insert(0, 'tools')
import glslc, isa64, float_ref as fr


def f24(x): return fr.encode_real(isa64.F(x).limit_denominator(1 << 14), isa64.FMT)
def real(b): return float(fr.to_real(b, isa64.FMT))


RANK = {'mat4': 0, 'mat2': 1, 'vec4': 2, 'mat3': 3, 'vec3': 4, 'vec2': 5, 'float': 6, 'int': 6, 'bool': 6}
ROWS = {'mat4': 4, 'mat3': 3, 'mat2': 2}


def place(vars, max_rows):
    """vars: (rank, rows). As g2sc_pack_vars: returns row * 4 + lane of each, and the rows in use"""
    order = sorted(range(len(vars)), key=lambda i: (vars[i][0], -vars[i][1]))
    used = [0] * max_rows
    out = [0] * len(vars)
    top = 0
    i = 0
    while i < len(order) and vars[order[i]][0] <= 4:
        v = order[i]; n = vars[v][1]
        assert top + n <= max_rows, 'does not fit'
        out[v] = top * 4
        for k in range(n): used[top + k] = 15 if vars[v][0] <= 2 else 7
        top += n; i += 1
    a01 = a23 = max_rows - top; nx = top; bot = max_rows
    while i < len(order) and vars[order[i]][0] == 5:
        v = order[i]; n = vars[v][1]
        if n <= a01:
            out[v] = nx * 4
            for k in range(n): used[nx + k] |= 3
            nx += n; a01 -= n
        else:
            assert n <= a23, 'does not fit'
            bot -= n; out[v] = bot * 4 + 2
            for k in range(n): used[bot + k] |= 12
            a23 -= n
        i += 1
    while i < len(order):
        v = order[i]; n = vars[v][1]
        best, col, at = max_rows + 1, -1, 0
        for c in range(4):
            start = 0; b, t = max_rows + 1, 0
            for r in range(max_rows + 1):
                if r < max_rows and not used[r] >> c & 1: continue
                if r - start >= n and r - start < b: b, t = r - start, start
                start = r + 1
            if b < best: best, col, at = b, c, t
        assert col >= 0, 'does not fit'
        out[v] = at * 4 + col
        for k in range(n): used[at + k] |= 1 << col
        i += 1
    return out, max((r + 1 for r in range(max_rows) if used[r]), default=0)


def table(tab, rows):
    """a stage's symbol table: name, slot, type, count -> the map of its slots"""
    vs = []
    for name, v in tab.items():
        vs.append((RANK[v[1]], ROWS.get(v[1], 1) * (v[2] if len(v) > 2 else 1), v[0]))
    at, n = place([(a, b) for a, b, _ in vs], rows)
    top = max((s + r for _, r, s in vs), default=0)
    m = [0] * top
    for (a, r, s), p in zip(vs, at):
        for k in range(r): m[s + k] = p + 4 * k
    return m, n


def fewest(tab):
    """as g2sc_link places varyings: in the fewest rows that hold them"""
    for rows in range(1, 9):
        try:
            return table(tab, rows)
        except AssertionError:
            pass
    raise AssertionError('does not fit')


def link(vsrc, fsrc, pack_u=False):
    v0 = glslc.compile_stage(vsrc, 'vs', isa64=True, forward=True)
    f0 = glslc.compile_stage(fsrc, 'fs', isa64=True, forward=True)
    vmap, _ = fewest(v0['varyings'])
    pv, pf = {'vmap': vmap}, {'vmap': vmap}
    if pack_u:
        pv['umap'], pv['urows'] = table(v0['uniforms'], 224) if v0['uniforms'] else ([], 0)
        pf['umap'], pf['urows'] = table(f0['uniforms'], 48) if f0['uniforms'] else ([], 0)
    return v0, f0, pv, pf


def params_of(result, pack, stage, vals):
    p = [[0, 0, 0, 0] for _ in range(256 if stage == 'vs' else 64)]
    for name, v in result['uniforms'].items():
        at0 = v[0]
        n = (v[2] if len(v) > 2 else 1) * ROWS.get(v[1], 1)
        w = {'float': 1, 'int': 1, 'vec2': 2, 'vec3': 3}.get(v[1], 4)
        for k in range(n):
            at = pack['umap'][at0 + k] if 'umap' in pack else at0 * 4 + k * 4
            for c in range(w): p[at >> 2][(at & 3) + c] = f24(vals(name, k, c))
    for i, c in enumerate(result['const_pool']): p[result['const_base'] + i] = [f24(x) for x in c]
    return p


def run(name, vsrc, fsrc, want, vals=lambda n, k, c: 0, pack_u=False, dump=None):
    v0, f0, pv, pf = link(vsrc, fsrc, pack_u)
    v = glslc.compile_stage(vsrc, 'vs', isa64=True, forward=True, pack=pv)
    f = glslc.compile_stage(fsrc, 'fs', isa64=True, forward=True, pack=pf)
    if dump:
        os.makedirs(dump, exist_ok=True)
        for ext, src, pk in (('vert', vsrc, pv), ('frag', fsrc, pf)):
            path = '%s/%s.%s' % (dump, name, ext)
            open(path, 'w', newline='').write(src)
            with open(path + '.pack', 'w') as o:
                if 'umap' in pk: o.write('U %d %d %s\n' % (pk['urows'], len(pk['umap']), ' '.join(map(str, pk['umap']))))
                o.write('V %d %s\n' % (len(pk['vmap']), ' '.join(map(str, pk['vmap']))))
        return 0
    assert v['const_base'] + len(v['const_pool']) <= 256 and f['const_base'] + len(f['const_pool']) <= 64, 'still too large'
    vw, _ = isa64.assemble(v['sasm'].splitlines(), vs=True)
    fw, _ = isa64.assemble(f['sasm'].splitlines(), vs=False)
    inp = isa64.default_vs_inputs(rich=True)
    inp.params = params_of(v0 | {'const_base': v['const_base'], 'const_pool': v['const_pool']}, pv, 'vs', vals)
    rv = isa64.simulate_vs2(vw, inp)
    varin = [[rv.outv[r][l] for r in range(8)] + [[0] * 4] * 8 for l in range(16)]
    rf = isa64.simulate_fs2(fw, isa64.FSInputs(params_of(f0 | {'const_base': f['const_base'], 'const_pool': f['const_pool']}, pf, 'fs', vals), varin, active=0xFFFF, interp=False))
    bad = 0
    for l in range(16):
        got = [real(c) for c in rf.outc[l]]
        if any(abs(a - b) > 1e-3 * max(1.0, abs(b)) for a, b in zip(got, want)) or rv.err or rf.err:
            print('FAIL %s: lane %d got %s, expected %s%s' % (name, l, got, want, ', core error' if rv.err or rf.err else ''))
            bad = 1
            break
    return bad


P = 'precision mediump float;\n'
A = 'attribute vec4 a0;\n'
n = 12
twelve_v = A + ''.join('varying vec2 v%d;\n' % i for i in range(n)) + 'void main() { gl_Position = a0;\n' + ''.join('  v%d = vec2(%d.0, %d.5);\n' % (i, i, i) for i in range(n)) + '}\n'
twelve_f = P + ''.join('varying vec2 v%d;\n' % i for i in range(n)) + 'void main() { vec2 s = vec2(0.0);\n' + ''.join('  s += v%d * %d.0;\n' % (i, i + 1) for i in range(n)) + '  gl_FragColor = vec4(s, v3.y, v11.x); }\n'
w = [sum((i + 1) * i for i in range(n)), sum((i + 1) * (i + .5) for i in range(n)), 3.5, 11]
mix_v = A + 'varying float a; varying vec3 b; varying vec2 c; varying float d; varying vec3 e; varying float f; varying float g; varying vec4 h; varying mat2 m; varying vec2 k[2]; varying float o[3];\n' \
    'void main() { gl_Position = a0; a = 1.0; b.xy = vec2(2.0, 3.0); b.z = b.y + 1.0; c = vec2(5.0, 6.0); d = 7.0; e = vec3(8.0, 9.0, 10.0); f = 11.0; g = 12.0; h = vec4(13.0, 14.0, 15.0, 16.0);\n' \
    '  m = mat2(17.0, 18.0, 19.0, 20.0); k[0] = vec2(21.0, 22.0); k[1] = vec2(23.0, 24.0); o[0] = 25.0; o[1] = 26.0; o[2] = 27.0; }\n'
mix_f = P + 'varying float a; varying vec3 b; varying vec2 c; varying float d; varying vec3 e; varying float f; varying float g; varying vec4 h; varying mat2 m; varying vec2 k[2]; varying float o[3];\n' \
    'void main() { gl_FragColor = vec4(a + b.x * 2.0 + b.y * 3.0 + b.z * 4.0 + c.x * 5.0 + c.y * 6.0 + d * 7.0, e.x + e.y * 2.0 + e.z * 3.0 + f * 4.0 + g * 5.0, h.w + m[1][0] + k[1].y + o[2], m[0][1] + k[0].x + o[1]); }\n'
# arrays of floats, each a column of the 8 rows
floats_v = A + 'varying float f[8]; varying float g[8]; varying float h[8]; varying float j[8];\nvoid main() { gl_Position = a0; for (int i = 0; i < 8; i++) { f[i] = float(i); g[i] = float(i + 8); h[i] = float(i + 16); j[i] = float(i + 24); } }\n'
floats_f = P + 'varying float f[8]; varying float g[8]; varying float h[8]; varying float j[8];\nvoid main() { gl_FragColor = vec4(f[0], f[7], g[0] + g[7], j[7] + h[1]); }\n'
# uniforms: floats past a row each, and the fragment stage reads them at constant indices, the vertex stage at a run-time one
uni_f = P + 'uniform float u[40]; uniform float q[40]; uniform vec2 w[30]; uniform vec3 x[4]; uniform vec4 y; varying vec2 t;\nvoid main() { gl_FragColor = vec4(u[0] + u[39] + q[1], w[29].y, x[3].z + y.w, t.x); }\n'
uni_v = A + 'uniform float s[200]; uniform vec2 r[60]; varying vec2 t;\nvoid main() { int i = int(a0.x * 0.0) + 199; int j = int(a0.y * 0.0) + 59; gl_Position = a0; t = vec2(s[i] + s[0], r[j].y); }\n'


# fewer rows than one a variable: a colour and two coordinates in two rows, a vec3 and a float in one, two vec2 of one row read in different blocks
coords_v = A + 'varying vec4 col; varying vec2 t0; varying vec2 t1;\nvoid main() { gl_Position = a0; col = vec4(0.5, 1.0, 2.0, 4.0); t0 = vec2(1.0, 2.0); t1 = vec2(3.0, 5.0); }\n'
coords_f = P + 'varying vec4 col; varying vec2 t0; varying vec2 t1;\nvoid main() { gl_FragColor = vec4(t0.x + t1.y, t0.y * t1.x, col.z + t1.x, col.w * t0.y); }\n'
v3f_v = A + 'varying vec3 n; varying float f;\nvoid main() { gl_Position = a0; n = vec3(1.0, 2.0, 3.0); f = 4.0; }\n'
v3f_f = P + 'varying vec3 n; varying float f;\nvoid main() { gl_FragColor = vec4(n.zyx, f); }\n'
apart_v = A + 'varying vec2 a; varying vec2 b;\nvoid main() { gl_Position = a0; a = vec2(1.0, 2.0); b = vec2(3.0, 4.0); }\n'
apart_f = P + 'varying vec2 a; varying vec2 b;\nvoid main() { vec4 c = vec4(a, 0.0, 1.0); if (a.x > 0.5) c.zw = b; gl_FragColor = c; }\n'


def uvals(name, k, c):
    return {'u': k, 'q': k * 2, 'w': k + c * .5, 'x': k + c, 'y': c + 1, 's': k, 'r': k + c * .25}[name] * 1.0


CASES = [
    ('twelve_vec2', twelve_v, twelve_f, w, None, False),
    ('every_shape', mix_v, mix_f, [1 + 4 + 9 + 16 + 25 + 36 + 49, 8 + 18 + 30 + 44 + 60, 16 + 19 + 24 + 27, 18 + 21 + 26], None, False),
    ('floats', floats_v, floats_f, [0, 7, 23, 31 + 17], None, False),
    ('uniforms', uni_v, uni_f, [41, 29.5, 3 + 2 + 4, 199], uvals, True),
    ('two_coords', coords_v, coords_f, [6, 6, 5, 8], None, False),
    ('vec3_float', v3f_v, v3f_f, [3, 2, 1, 4], None, False),
    ('read_apart', apart_v, apart_f, [1, 2, 3, 4], None, False),
]


def main():
    d = sys.argv[2] if len(sys.argv) > 2 and sys.argv[1] == '--dump' else None
    bad = 0
    for name, v, f, want, vals, pu in CASES:
        bad += run(name, v, f, want, vals or (lambda n, k, c: 0), pu, d)
    if d:
        return 0
    # the packed shapes: the rows the link takes, and the LDVs of the fragment program (one a row in a block; read_apart's second is inside its IF)
    for name, vsrc, fsrc, rows, ldvs in (('two_coords', coords_v, coords_f, 2, 2), ('vec3_float', v3f_v, v3f_f, 1, 1), ('read_apart', apart_v, apart_f, 1, 2)):
        v0, f0, pv, pf = link(vsrc, fsrc)
        got = 1 + max(pv['vmap']) // 4
        n = sum(1 for ln in glslc.compile_stage(fsrc, 'fs', isa64=True, forward=True, pack=pf)['sasm'].splitlines() if ln.startswith('LDV'))
        if got != rows or n != ldvs:
            print('FAIL %s: %d rows, %d LDVs; expected %d, %d' % (name, got, n, rows, ldvs))
            bad += 1
    # the limits of the placement
    for what, vs, rows, fits in (('32 floats in 8 rows', [(6, 1)] * 32, 8, True), ('33 floats', [(6, 1)] * 33, 8, False), ('8 vec3 and 8 floats', [(4, 1)] * 8 + [(6, 1)] * 8, 8, True),
                                 ('16 vec2', [(5, 1)] * 16, 8, True), ('17 vec2', [(5, 1)] * 17, 8, False), ('4 mat2', [(1, 2)] * 4, 8, True), ('5 mat2', [(1, 2)] * 5, 8, False)):
        try:
            place(vs, rows)
            ok = True
        except AssertionError:
            ok = False
        if ok != fits:
            print('FAIL placement: %s' % what)
            bad += 1
    print('%d programs, %d failures' % (len(CASES), bad))
    return 1 if bad else 0


sys.exit(main())
