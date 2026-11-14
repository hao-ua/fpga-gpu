#!/usr/bin/env python3
"""flow_check — per-lane control flow against the truth: every shader below runs in the core simulator on a warp whose lanes disagree (odd lanes x = 0.25, even x = 0.75), and each lane's result is compared with the answer worked out by hand. opt_check compares two compilations; this one catches what both get wrong.
    python3 -I host/win/shc/test/flow_check.py      (run from the repository root) """
import sys
sys.path.insert(0, 'tools')
import glslc, isa64, float_ref as fr


def f24(x): return fr.encode_real(isa64.F(x).limit_denominator(1 << 14), isa64.FMT)
def real(b): return float(fr.to_real(b, isa64.FMT))


FS = 'precision mediump float; varying vec4 v0; uniform float n;\nstruct S { vec4 a; float b; };\nstruct T { mat2 m; float c; };\n'
VS = 'attribute vec4 a0; varying vec4 v0; uniform float n;\n'
# name, stage, source, (result where x > 0.5, result elsewhere)
CASES = [
    ('function return under if', 'fs', FS + 'float f(float x) { if (x > 0.5) return 1.0; return 2.0; }\nvoid main() { gl_FragColor = vec4(f(v0.x)); }\n', (1, 2)),
    ('return in both branches', 'fs', FS + 'float f(float x) { if (x > 0.5) { return 1.0; } else { return 2.0; } }\nvoid main() { gl_FragColor = vec4(f(v0.x)); }\n', (1, 2)),
    ('code between the returns', 'fs', FS + 'float f(float x) { float r = 3.0; if (x > 0.5) { r = 1.0; return r; } r = 2.0; return r; }\nvoid main() { gl_FragColor = vec4(f(v0.x)); }\n', (1, 2)),
    ('three returns', 'fs', FS + 'float f(float x) { if (x > 0.9) return 9.0; if (x > 0.5) return 1.0; if (x > 0.1) return 2.0; return 8.0; }\nvoid main() { gl_FragColor = vec4(f(v0.x)); }\n', (1, 2)),
    ('out argument, early return', 'fs', FS + 'void f(out vec4 c, float x) { if (x > 0.5) { c = vec4(1.0); return; } c = vec4(2.0); }\nvoid main() { vec4 q; f(q, v0.x); gl_FragColor = q; }\n', (1, 2)),
    ('a call in an if', 'fs', FS + 'float f(float x) { if (x > 0.5) return 1.0; return 2.0; }\nvoid main() { float r = 5.0; if (v0.x > 0.0) r = f(v0.x); gl_FragColor = vec4(r); }\n', (1, 2)),
    ('nested calls', 'fs', FS + 'float g(float x) { if (x > 0.5) return 1.0; return 2.0; }\nfloat f(float x) { if (x > 2.0) return 7.0; float y = g(x); if (y > 1.5) return y; return y; }\nvoid main() { gl_FragColor = vec4(f(v0.x)); }\n', (1, 2)),
    ('return in a hardware loop', 'fs', FS + 'float f(float x) { for (float i = 0.0; i < n; i += 1.0) { if (x > 0.5) return 1.0; } return 2.0; }\nvoid main() { gl_FragColor = vec4(f(v0.x)); }\n', (1, 2)),
    ('return past a nested loop', 'fs', FS + 'float f(float x) { float s = 0.0; for (float i = 0.0; i < n; i += 1.0) { for (float j = 0.0; j < n; j += 1.0) { if (x > 0.5) return 1.0; } s = 2.0; } return s; }\nvoid main() { gl_FragColor = vec4(f(v0.x)); }\n', (1, 2)),
    ('return in an unrolled loop', 'fs', FS + 'float f(float x) { float s = 2.0; for (int i = 0; i < 3; i++) { if (x > 0.5) return 1.0; s = 2.0; } return s; }\nvoid main() { gl_FragColor = vec4(f(v0.x)); }\n', (1, 2)),
    ('matrix return per lane', 'fs', FS + 'mat2 f(float x) { if (x > 0.5) return mat2(1.0, 1.0, 1.0, 1.0); return mat2(2.0, 2.0, 2.0, 2.0); }\nvoid main() { mat2 m = f(v0.x); gl_FragColor = vec4(m[1].y); }\n', (1, 2)),
    ('struct return per lane', 'fs', FS + 'S f(float x) { S a = S(vec4(1.0), 1.0); S b = S(vec4(2.0), 2.0); if (x > 0.5) return a; return b; }\nvoid main() { S s = f(v0.x); gl_FragColor = vec4(s.b); }\n', (1, 2)),
    ('main: return under if', 'fs', FS + 'void main() { gl_FragColor = vec4(1.0); if (v0.x > 0.5) return; gl_FragColor = vec4(2.0); }\n', (1, 2)),
    ('main: return past the rest', 'fs', FS + 'void main() { gl_FragColor = vec4(1.0); return; gl_FragColor = vec4(2.0); }\n', (1, 1)),
    ('main: return with a partial write', 'fs', FS + 'void main() { gl_FragColor.rgb = vec3(1.0); gl_FragColor.a = 1.0; if (v0.x > 0.5) return; gl_FragColor.r = 2.0; }\n', (1, 2)),
    ('gl_FragColor in part', 'fs', FS + 'void main() { gl_FragColor.gba = vec3(0.0); gl_FragColor.r = v0.x > 0.5 ? 1.0 : 2.0; }\n', (1, 2)),
    ('gl_FragColor in part, in an if', 'fs', FS + 'void main() { gl_FragColor = vec4(2.0); if (v0.x > 0.5) gl_FragColor.r = 1.0; }\n', (1, 2)),
    ('gl_FragColor read back', 'fs', FS + 'void main() { gl_FragColor = vec4(2.0); if (v0.x > 0.5) gl_FragColor = gl_FragColor * 0.5; }\n', (1, 2)),
    ('gl_FragColor compound', 'fs', FS + 'void main() { gl_FragColor = vec4(v0.x > 0.5 ? 1.0 : 2.0); gl_FragColor *= 1.0; }\n', (1, 2)),
    ('gl_FragData in part', 'fs', FS + 'void main() { gl_FragData[0].r = v0.x > 0.5 ? 1.0 : 2.0; gl_FragData[0].gba = vec3(0.0); }\n', (1, 2)),
    ('gl_FragColor from a function', 'fs', FS + 'void f(float x) { gl_FragColor.r = x; } void main() { gl_FragColor.gba = vec3(0.0); f(v0.x > 0.5 ? 1.0 : 2.0); }\n', (1, 2)),
    ('gl_FragColor as an out argument', 'fs', FS + 'void f(out vec4 c, float x) { if (x > 0.5) { c = vec4(1.0); return; } c = vec4(2.0); }\nvoid main() { f(gl_FragColor, v0.x); }\n', (1, 2)),
    ('matrix assigned in an if', 'fs', FS + 'void main() { mat2 m = mat2(2.0, 2.0, 2.0, 2.0); mat2 k = mat2(1.0, 1.0, 1.0, 1.0); if (v0.x > 0.5) m = k; gl_FragColor = vec4(m[0].x); }\n', (1, 2)),
    ('a matrix copy shares nothing', 'fs', FS + 'void main() { mat2 m = mat2(1.0, 1.0, 1.0, 1.0); mat2 k = m; k[0] = vec2(2.0); gl_FragColor = vec4(v0.x > 0.5 ? m[0].x : k[0].x); }\n', (1, 2)),
    ('a matrix argument is a copy', 'fs', FS + 'float f(mat2 q) { q[0] = vec2(2.0); return q[0].x; }\nvoid main() { mat2 m = mat2(1.0, 1.0, 1.0, 1.0); float r = f(m); gl_FragColor = vec4(v0.x > 0.5 ? m[0].x : r); }\n', (1, 2)),
    ('struct assigned in an if', 'fs', FS + 'void main() { S s = S(vec4(2.0), 2.0); S t = S(vec4(1.0), 1.0); if (v0.x > 0.5) s = t; gl_FragColor = vec4(s.a.x); }\n', (1, 2)),
    ('struct with a matrix, in an if', 'fs', FS + 'void main() { T s = T(mat2(2.0, 2.0, 2.0, 2.0), 2.0); T t = T(mat2(1.0, 1.0, 1.0, 1.0), 1.0); if (v0.x > 0.5) s = t; gl_FragColor = vec4(s.m[1].x); }\n', (1, 2)),
    ('matrix member in an if', 'fs', FS + 'void main() { T s = T(mat2(2.0, 2.0, 2.0, 2.0), 2.0); if (v0.x > 0.5) s.m = mat2(1.0, 1.0, 1.0, 1.0); gl_FragColor = vec4(s.m[0].y); }\n', (1, 2)),
    ('a struct copy shares nothing', 'fs', FS + 'void main() { T s = T(mat2(1.0, 1.0, 1.0, 1.0), 1.0); T t = s; t.m[0] = vec2(2.0); gl_FragColor = vec4(v0.x > 0.5 ? s.m[0].x : t.m[0].x); }\n', (1, 2)),
    ('m = m * n', 'fs', FS + 'void main() { mat2 m = mat2(1.0, 0.0, 0.0, 1.0); m = m * mat2(2.0, 0.0, 0.0, 2.0); gl_FragColor = vec4(v0.x > 0.5 ? 1.0 : m[0].x); }\n', (1, 2)),
    ('?: of matrices', 'fs', FS + 'void main() { mat2 m = v0.x > 0.5 ? mat2(1.0, 1.0, 1.0, 1.0) : mat2(2.0, 2.0, 2.0, 2.0); gl_FragColor = vec4(m[1].x); }\n', (1, 2)),
    ('?: of structs', 'fs', FS + 'void main() { S a = S(vec4(1.0), 1.0); S b = S(vec4(2.0), 2.0); S s = v0.x > 0.5 ? a : b; gl_FragColor = vec4(s.b); }\n', (1, 2)),
    ('VS: function return under if', 'vs', VS + 'float f(float x) { if (x > 0.5) return 1.0; return 2.0; }\nvoid main() { gl_Position = vec4(f(a0.x)); }\n', (1, 2)),
    ('VS: main returns, varying kept', 'vs', VS + 'void main() { v0.xy = a0.xy; v0.zw = a0.zw; gl_Position = vec4(1.0); if (a0.x > 0.5) return; gl_Position = vec4(2.0); }\n', (1, 2)),
    ('VS: gl_Position in part', 'vs', VS + 'void main() { gl_Position = vec4(1.0); gl_Position.y = 0.0; gl_Position.x = a0.x > 0.5 ? 1.0 : 2.0; }\n', (1, 2)),
    ('VS: gl_Position read back', 'vs', VS + 'void main() { gl_Position = vec4(a0.x > 0.5 ? 0.5 : 1.0); gl_Position.x = gl_Position.y * 2.0; }\n', (1, 2)),
    ('VS: gl_PointSize read back', 'vs', VS + 'void main() { gl_PointSize = a0.x > 0.5 ? 1.0 : 2.0; gl_Position = vec4(gl_PointSize); }\n', (1, 2)),
]
OUT = 'gl_FragColor = vec4(%s); }\n'
# names: functions are inlined, and each must still see only its own. n = 4, v0.x = 0.75 or 0.25
CASES += [(name, 'fs', FS + src, want) for name, src, want in (
    ('callee local, caller constant', 'float f() { float x = 2.0; return x; }\nvoid main() { float x = 1.0; float y = f(); ' + OUT % 'x + y', (3, 3)),
    ('global: read in a function', 'float g = 2.0;\nfloat f() { return g; }\nvoid main() { ' + OUT % 'f() + 1.0', (3, 3)),
    ('global: written in a function, under an if', 'float g = 1.0;\nvoid f() { g = 2.0; }\nvoid main() { if (v0.x < 0.5) f(); ' + OUT % 'g', (1, 2)),
    ('global: a local hides it', 'float g = 1.0;\nfloat f() { return g; }\nvoid main() { float g = 5.0; ' + OUT % 'f() + g', (6, 6)),
    ('global: no initializer, written in main', 'vec4 g;\nfloat f() { return g.y; }\nvoid main() { g = v0; g.y = v0.x > 0.5 ? 1.0 : 2.0; ' + OUT % 'f()', (1, 2)),
    ('global: two on one line, in a loop', 'float a = 1.0, b = 0.0;\nvoid f() { b = b + a; }\nvoid main() { for (int i = 0; i < 2; i++) { if (v0.x < 0.5) f(); } ' + OUT % 'b + 1.0', (1, 3)),
    ('global: an array', 'float a[2];\nvoid f(float x) { a[0] = x; a[1] = x + 1.0; }\nvoid main() { f(1.0); ' + OUT % 'v0.x > 0.5 ? a[0] : a[1]', (1, 2)),
    ('parameter, caller constant', 'float f(float x) { return x; }\nvoid main() { float x = 1.0; ' + OUT % 'f(2.0) + x', (3, 3)),
    ('parameter, caller register', 'float f(float x) { return x * 2.0; }\nvoid main() { float x = v0.x; ' + OUT % 'f(1.0) + x', (2.75, 2.25)),
    ('uniform, caller constant', 'float f() { return n; }\nvoid main() { float n = 1.0; ' + OUT % 'f() + n', (5, 5)),
    ('uniform, caller register', 'float f() { return n; }\nvoid main() { float n = v0.x; ' + OUT % 'f() + n', (4.75, 4.25)),
    ('varying, caller local', 'float f() { return v0.x; }\nvoid main() { vec4 v0 = vec4(1.0); ' + OUT % 'f() + v0.x', (1.75, 1.25)),
    ('global const, caller constant', 'const float c = 2.0;\nfloat f() { return c; }\nvoid main() { float c = 1.0; ' + OUT % 'f() + c', (3, 3)),
    ('global const, caller register', 'const float c = 2.0;\nfloat f() { return c; }\nvoid main() { float c = v0.x; ' + OUT % 'f() + c', (2.75, 2.25)),
    ('callee promotes, caller constant', 'float f(float a) { float x = 1.0; x = x + a; return x; }\nvoid main() { float x = 3.0; float y = f(v0.x); ' + OUT % 'x + y', (4.75, 4.25)),
    ('callee promotes, caller register', 'float f(float a) { float x = 1.0; x = x + a; return x; }\nvoid main() { float x = v0.x; float y = f(v0.x); ' + OUT % 'x + y', (2.5, 1.5)),
    ('out argument of a constant', 'void f(out float x) { x = v0.x; }\nvoid main() { float y = 1.0; f(y); ' + OUT % 'y * 2.0', (1.5, 0.5)),
    ('out argument under an if', 'void f(out float x) { x = 2.0; }\nvoid main() { float y = 1.0; if (v0.x < 0.5) f(y); ' + OUT % 'y', (1, 2)),
    ('a write inside an expression', 'void main() { float y = 1.0; float z = 0.0; if (v0.x < 0.5) z = (y = 2.0) + 1.0; ' + OUT % 'y', (1, 2)),
    ('a counter in a condition', 'void main() { float y = 1.0; if (v0.x < 0.5) { if (y++ > 0.0) { } } ' + OUT % 'y', (1, 2)),
    ('block hides a constant', 'void main() { float x = 1.0; { float x = 2.0; } ' + OUT % 'x', (1, 1)),
    ('block hides, then writes', 'void main() { float x = 1.0; { float x = 2.0; x = 5.0; } ' + OUT % 'x', (1, 1)),
    ('constant hides a register', 'void main() { float x = v0.x; { float x = 2.0; } ' + OUT % 'x', (0.75, 0.25)),
    ('register hides a constant', 'void main() { float x = 1.0; float r = 0.0; { float x = v0.x; r = x; } ' + OUT % 'x + r', (1.75, 1.25)),
    ('if block hides', 'void main() { float x = 1.0; if (v0.x > 0.0) { float x = 2.0; } ' + OUT % 'x', (1, 1)),
    ('a block writes the outer one', 'void main() { float x = 1.0; { x = 2.0; } ' + OUT % 'x', (2, 2)),
    ('promoted inside a block', 'void main() { float x = 1.0; float r = 0.0; { float x = 2.0; x = v0.x; r = x; } ' + OUT % 'r + x', (1.75, 1.25)),
    ('conditional write after a block', 'void main() { float q = 1.0; { float q = 5.0; } if (v0.x < 0.5) q = 2.0; ' + OUT % 'q', (1, 2)),
    ('conditional write in a block', 'void main() { float q = 1.0; { float q = 5.0; if (v0.x < 0.5) q = 7.0; } ' + OUT % 'q', (1, 1)),
    ('for variable hides a constant', 'void main() { float i = 5.0; for (int i = 0; i < 2; i++) { } ' + OUT % 'i', (5, 5)),
    ('for body hides a constant', 'void main() { float x = 1.0; for (int i = 0; i < 2; i++) { float x = 2.0; } ' + OUT % 'x', (1, 1)),
    ('initializer sees the outer name', 'void main() { float x = 1.0; float r = 0.0; { float x = x + 1.0; r = x; } ' + OUT % 'r + x', (3, 3)),
    ('initializer, outer register', 'void main() { float x = v0.x; float r = 0.0; { float x = x * 2.0; r = x; } ' + OUT % 'r + x', (2.25, 0.75)),
    ('loops with one name, two functions', 'float a(float x) { float s = 0.0; for (float i = 0.0; i < n; i += 1.0) { s += x; } return s; }\nfloat b(float x) { float s = 0.0; for (int i = 0; i < 3; i++) { s += x; } return s; }\nvoid main() { ' + OUT % 'a(1.0) + b(1.0)', (7, 7)),
    ('loops with one name, one function', 'void main() { float s = 0.0; for (float i = 0.0; i < n; i += 1.0) { s += 1.0; } for (int i = 0; i < 3; i++) { s += 1.0; } ' + OUT % 's', (7, 7)),
    ('one name conditional elsewhere', 'float a(float x) { float i = 1.0; if (x > 0.5) i = 2.0; return i; }\nvoid main() { float s = 0.0; for (int i = 0; i < 2; i++) { s += 1.0; } ' + OUT % 's + a(v0.x)', (4, 3)),
)]
# long constant loops (glslc._long_for): more than 16 passes with a break / continue, more than 64 without, are hardware loops with a per-lane BREAK and the index in a register — unless the
# body needs the index as a constant (an array index, an argument of a function of the shader), which keeps the loop unrolled
CASES += [(name, 'fs', FS + src, want) for name, src, want in (
    ('long loop: break per lane', 'void main() { float s = 0.0; for (int i = 0; i < 40; i++) { s += 1.0; if (v0.x > 0.5 && s > 2.5) break; } ' + OUT % 's', (3, 40)),
    ('long loop: continue per lane', 'void main() { float s = 0.0; for (int i = 0; i < 20; i++) { if (v0.x > 0.5) continue; s += 1.0; } ' + OUT % 's', (0, 20)),
    ('long loop: the index as a value', 'void main() { float s = 0.0; for (int i = 0; i < 30; i++) { s = float(i); if (v0.x > 0.5 && i == 4) break; } ' + OUT % 's', (4, 29)),
    ('long loop: no break, 70 passes', 'void main() { float s = 0.0; for (int i = 0; i < 70; i++) { s += v0.x > 0.5 ? 1.0 : 2.0; } ' + OUT % 's', (70, 140)),
    ('long loop: return and break', 'float f(float x) { float s = 0.0; for (int i = 0; i < 48; i++) { s += 1.0; if (x > 0.5 && s > 4.5) return s; if (s > 9.5) break; } return s + 100.0; }\nvoid main() { ' + OUT % 'f(v0.x)', (5, 110)),
    ('long loop: down from a global const', 'const int N = 33;\nvoid main() { float s = 0.0; for (int i = N; i > 0; i--) { s += 1.0; if (v0.x > 0.5) break; } ' + OUT % 's', (1, 33)),
    ('long loop: float index, step 0.5', 'void main() { float s = 0.0; for (float t = 0.0; t < 10.0; t += 0.5) { s += 1.0; if (v0.x > 0.5 && t > 1.75) break; } ' + OUT % 's', (5, 20)),
    ('long loop: nested, both break', 'void main() { float s = 0.0; for (int i = 0; i < 20; i++) { for (int j = 0; j < 20; j++) { s += 1.0; if (v0.x > 0.5 && j == 1) break; } if (s > 9.5) break; } ' + OUT % 's', (10, 20)),
    ('long loop: an index keeps it unrolled', 'void main() { float a[20]; for (int i = 0; i < 20; i++) { a[i] = 0.0; } for (int i = 0; i < 20; i++) { a[i] = 1.0; if (v0.x > 0.5 && i == 4) break; } float s = 0.0; for (int i = 0; i < 20; i++) { s += a[i]; } ' + OUT % 's', (5, 20)),
    ('long loop: its index handed to a function', 'float g(int k) { float a[3]; a[0] = 1.0; a[1] = 2.0; a[2] = 4.0; return k < 3 ? a[2] : 1.0; }\nvoid main() { float s = 0.0; for (int i = 0; i < 20; i++) { s += g(i); if (v0.x > 0.5 && i == 1) break; } ' + OUT % 's', (8, 29)),
    ('long loop: a value written before it, read after', 'void main() { float k = 2.0; float s = 0.0; for (int i = 0; i < 17; i++) { if (v0.x > 0.5) break; k = 3.0; s += k; } ' + OUT % 's + k', (2, 54)),
    ('long loop: the local of the same name after it', 'void main() { float i = 7.0; float s = 0.0; for (int i = 0; i < 18; i++) { s += 1.0; if (v0.x > 0.5) break; } ' + OUT % 's + i', (8, 25)),
    ('16 passes with a break stay unrolled', 'void main() { float s = 0.0; for (int i = 0; i < 16; i++) { s += 1.0; if (v0.x > 0.5 && i == 2) break; } ' + OUT % 's', (3, 16)),
)]
# counted loops and lone break / continue (glslc._counted_loop / _ctl_if): a long loop whose body never reads its index is `LOOP n`
# with no counter; `if (c) break;` is SETP c + BREAK, `a || b` a pair each. And the shapes that must keep the counter or the IF
CASES += [(name, 'fs', FS + src, want) for name, src, want in (
    ('counted: || breaks, either side', 'void main() { float s = 0.0; for (int i = 0; i < 40; i++) { s += 1.0; if (s > 30.5 || (v0.x > 0.5 && s > 2.5)) break; } ' + OUT % 's', (3, 31)),
    ('counted: || short-circuits a write', 'void main() { float k = 0.0; for (int i = 0; i < 20; i++) { if (v0.x > 0.5 || (k += 1.0) > 100.0) break; } ' + OUT % 'k', (0, 20)),
    ('counted: three-way ||', 'void main() { float s = 0.0; for (int i = 0; i < 30; i++) { s += 1.0; if (s > 25.5 || (v0.x > 0.5 && s > 1.5) || s < 0.0) break; } ' + OUT % 's', (2, 26)),
    ('counted: || continue', 'void main() { float s = 0.0; float k = 0.0; for (int i = 0; i < 20; i++) { k += 1.0; if (v0.x > 0.5 || k > 15.5) continue; s += 1.0; } ' + OUT % 's + k', (20, 35)),
    ('counted: break in a block of one', 'void main() { float s = 0.0; for (int i = 0; i < 40; i++) { s += 1.0; if (v0.x > 0.5 && s > 2.5) { { break; } } } ' + OUT % 's', (3, 40)),
    ('counted: break beside a write stays an IF', 'void main() { float s = 0.0; for (int i = 0; i < 30; i++) { if (v0.x > 0.5 && s > 1.5) { s += 10.0; break; } s += 1.0; } ' + OUT % 's', (12, 30)),
    ('counted: break with an else stays an IF', 'void main() { float s = 0.0; for (int i = 0; i < 40; i++) { if (v0.x > 0.5 && s > 2.5) break; else s += 1.0; } ' + OUT % 's', (3, 40)),
    ('counted: lone break inside an if', 'void main() { float s = 0.0; for (int i = 0; i < 40; i++) { if (v0.x > 0.5) { if (s > 2.5) break; s += 1.0; } else s += 2.0; } ' + OUT % 's', (3, 80)),
    ('counted: continue then work', 'void main() { float s = 0.0; float t = 0.0; for (int i = 0; i < 24; i++) { t += 1.0; if (v0.x > 0.5 && t > 20.5) continue; s += 1.0; } ' + OUT % 's', (20, 24)),
    ('counted: the index name again inside keeps the counter', 'void main() { float s = 0.0; for (int i = 0; i < 20; i++) { for (int i = 0; i < 2; i++) { s += 1.0; } if (v0.x > 0.5 && s > 3.5) break; } ' + OUT % 's', (4, 40)),
    ('counted: a write to the index keeps the counter', 'void main() { float s = 0.0; for (int i = 0; i < 40; i++) { s += 1.0; if (v0.x > 0.5 && s > 2.5) { i = 100; } } ' + OUT % 's', (3, 40)),
    ('counted: a parameter hides the bound', 'const int N = 33;\nfloat f(float x, int N) { float s = 0.0; for (int i = 0; i < N; i++) { s += 1.0; if (x > 0.5 && s > 1.5) break; } return s; }\nvoid main() { ' + OUT % 'f(v0.x, 20)', (2, 20)),
    ('counted: 100 passes, return inside', 'float f(float x) { float t = 0.0; for (int i = 0; i < 100; i++) { t += 1.0; if (x > 0.5 && t > 6.5) return t; } return t + 0.5; }\nvoid main() { ' + OUT % 'f(v0.x)', (7, 100.5)),
)]
# loop-invariant code motion (glslc.licm): what a hardware loop works out the same every pass runs once before it, and the
# shapes it must leave: a value read after the loop, an accumulator, work under an IF, a value read in a pass before it is written
CASES += [(name, 'fs', FS + src, want) for name, src, want in (
    ('licm: a constant vector, break first', 'void main() { float s = 0.0; for (int i = 0; i < 30; i++) { if (v0.x > 0.5) break; vec3 c = vec3(1.0, 2.0, 3.0) * n; s += c.y; } ' + OUT % 's', (0, 240)),
    ('licm: read after the loop stays', 'void main() { float s = 0.0; float k = 1.0; for (int i = 0; i < 20; i++) { if (v0.x > 0.5 && s > 1.5) break; k = n * 2.0; s += 1.0; } ' + OUT % 's + k', (10, 28)),
    ('licm: an accumulator stays', 'void main() { float s = 0.0; for (int i = 0; i < 20; i++) { s = s + n; if (v0.x > 0.5 && s > 10.0) break; } ' + OUT % 's', (12, 80)),
    ('licm: read in a pass before the write stays', 'void main() { float s = 0.0; float t = 0.0; for (int i = 0; i < 20; i++) { s += t; t = n * 2.0; if (v0.x > 0.5 && s > 10.0) break; } ' + OUT % 's', (16, 152)),
    ('licm: under an IF stays', 'void main() { float s = 0.0; for (int i = 0; i < 20; i++) { if (v0.x > 0.5) { float c = n * 0.5; s += c; } else s += 1.0; } ' + OUT % 's', (40, 20)),
    ('licm: from an inner loop out of both', 'void main() { float s = 0.0; for (int i = 0; i < 20; i++) { for (int j = 0; j < 20; j++) { s += dot(vec2(1.0, 2.0), vec2(n)); if (v0.x > 0.5 && j == 1) break; } if (v0.x > 0.5) break; } ' + OUT % 's', (24, 4800)),
    ('licm: a runtime loop', 'void main() { float s = 0.0; for (float i = 0.0; i < n; i += 1.0) { vec2 c = vec2(3.0, 5.0) * n; s += c.x - c.y; if (v0.x > 0.5) break; } ' + OUT % 's', (-8, -32)),
)]


# a loop CONDITION that writes (a Shadertoy source's `for (..; i++ < 2e1; )` with `float i = 0.`): the counter is a
# tracked constant that nothing in post or body writes, and it must still become a register — else the condition compares
# the same constant in every pass and the loop runs to the hardware's 4096-pass cap
CASES += [(name, 'fs', FS + src, want) for name, src, want in (
    ('for: the condition counts, constant start', 'void main() { float i = 0.0; float s = 0.0; for (; i++ < 5.0; ) { s += v0.x > 0.5 ? 1.0 : 2.0; } ' + OUT % 's', (5, 10)),
    ('for: the condition counts, value after the loop', 'void main() { float i = 0.0; float s = 0.0; for (s = 1.0; i++ < 3.0; ) { s += 1.0; } ' + OUT % 's + i', (8, 8)),
    ('while: the condition counts', 'void main() { float i = 0.0; float s = 0.0; while (i++ < 4.0) { s += v0.x > 0.5 ? 1.0 : 2.0; } ' + OUT % 's + i', (9, 13)),
    ('do-while: the condition counts', 'void main() { float i = 0.0; float s = 0.0; do { s += 1.0; } while (i++ < 3.0); ' + OUT % 's + i', (8, 8)),
    ('nested: both conditions count', 'void main() { float i = 0.0; float d = 0.0; float s = 0.0; for (; i++ < 3.0; ) { for (d = 0.0; d++ < 2.0; ) { s += v0.x > 0.5 ? 1.0 : 2.0; } } ' + OUT % 's', (6, 12)),
    ('int counter in the condition', 'void main() { int i = 0; float s = 0.0; for (; i++ < 6; ) { s += 1.0; if (v0.x > 0.5 && i == 2) break; } ' + OUT % 's', (2, 6)),
)]
# do { .. continue; .. } while (cond): the condition is tested at the loop top (a lane's CONT holds it until ENDL), and
# worked out only where a pass has run — working it out before the first pass too, its result ignored, would be one
# evaluation too many when it writes
CASES += [(name, 'fs', FS + src, want) for name, src, want in (
    ('do-while: continue, the condition writes', 'void main() { float s = 0.0; float k = 0.0; do { s += 1.0; if (v0.x > 0.5) continue; s += 10.0; } while ((k += 1.0) < 3.0); ' + OUT % 's + k * 100.0', (303, 333)),
    ('do-while: continue, the condition reads', 'void main() { float s = 0.0; float i = 0.0; do { i += 1.0; if (v0.x > 0.5 && i < 3.0) continue; s += 1.0; } while (i < 5.0); ' + OUT % 's + i', (8, 10)),
    ('do-while: continue, lanes leave apart', 'void main() { float k = 0.0; do { k += 1.0; if (v0.x > 0.5) continue; } while (k < (v0.x > 0.5 ? 2.0 : 4.0)); ' + OUT % 'k', (2, 4)),
    ('do-while: continue, a counter in the condition', 'void main() { float i = 0.0; float s = 0.0; do { if (v0.x > 0.5) continue; s += 1.0; } while (i++ < 3.0); ' + OUT % 's * 10.0 + i', (4, 44)),
)]


# a variable declared without a value starts at 0. GLSL ES leaves the read undefined; WebGL zeroes them and
# sources written there rely on it (`float z, d, i; for (O *= i; i++ < 2e1; )`); unzeroed they read what the register last
# held: a local of a loop body keeps the last pass's value, an out parameter the caller's
CASES += [(name, 'fs', FS + src, want) for name, src, want in (
    ('zero: a loop local, every pass', 'void main() { float s = 0.0; for (int k = 0; k < 3; k++) { float a; a += v0.x > 0.5 ? 1.0 : 2.0; s += a; } ' + OUT % 's', (3, 6)),
    ('zero: a loop local in a runtime loop', 'void main() { float s = 0.0; for (float k = 0.0; k < n; k += 1.0) { float a; a += v0.x > 0.5 ? 1.0 : 2.0; s += a; if (k > 1.5) break; } ' + OUT % 's', (3, 6)),
    ('zero: a vector, by components', 'void main() { float s = 0.0; for (int k = 0; k < 3; k++) { vec3 p; p.x += 1.0; p.y += v0.x > 0.5 ? 1.0 : 2.0; s += p.x + p.y + p.z; } ' + OUT % 's', (6, 9)),
    ('zero: an int', 'void main() { float s = 0.0; for (int k = 0; k < 4; k++) { int c; c += 2; s += float(c); } ' + OUT % 's', (8, 8)),
    ('zero: under an if', 'void main() { float s = 0.0; for (int k = 0; k < 3; k++) { if (v0.x > 0.5) { float a; a += 2.0; s += a; } else { float b; b += 1.0; s += b * 10.0; } } ' + OUT % 's', (6, 30)),
    ('zero: an out parameter', 'void f(out float o) { o += 3.0; } void main() { float x = 5.0; if (v0.x > 0.5) f(x); ' + OUT % 'x', (3, 5)),
    ('zero: out and in the same variable', 'void f(float a, out float o) { o += a; } void main() { float x = 5.0; f(x, x); ' + OUT % 'x', (5, 5)),
    ('zero: an out parameter, part of a vector', 'void f(out float o) { o += 3.0; } void main() { vec2 v = vec2(5.0); f(v.y); ' + OUT % 'v.x + v.y', (8, 8)),
    ('zero: an out vector written in part', 'void f(out vec2 o) { o.x = 1.0; } void main() { vec2 v = vec2(5.0); f(v); ' + OUT % 'v.x + v.y', (1, 1)),
    ('zero: a struct', 'struct S { float a; vec2 b; }; void main() { float s = 0.0; for (int k = 0; k < 2; k++) { S q; q.a += 1.0; q.b.y += 2.0; s += q.a + q.b.y + q.b.x; } ' + OUT % 's', (6, 6)),
    ('zero: an array', 'void main() { float s = 0.0; for (int k = 0; k < 3; k++) { float a[3]; a[1] += 1.0; s += a[0] + a[1] + a[2]; } ' + OUT % 's', (3, 3)),
    ('zero: a matrix', 'void main() { float s = 0.0; for (int k = 0; k < 3; k++) { mat2 m; m[1] += vec2(1.0, 2.0); s += m[0].x + m[1].y; } ' + OUT % 's', (6, 6)),
    ('zero: a global', 'float g; void main() { for (int k = 0; k < 3; k++) g += v0.x > 0.5 ? 1.0 : 2.0; ' + OUT % 'g', (3, 6)),
    ('zero: the Shadertoy form', 'void main() { float z, d, i; float s = 0.0; for (s *= i; i++ < 4.0; ) { for (d = 0.0; d++ < 2.0; ) z += v0.x > 0.5 ? 1.0 : 2.0; s += z; } ' + OUT % 's + i', (25, 45)),
)]


def main():
    bad = 0
    for name, stage, src, want in CASES:
        try:
            r = glslc.compile_stage(src, stage, isa64=True, forward=True)
            plain = glslc.compile_stage(src, stage, isa64=True)
            results = []
            for k in (r, plain):
                words, _ = isa64.assemble(k['sasm'].splitlines(), vs=stage == 'vs')
                params = [[0, 0, 0, 0] for _ in range(256 if stage == 'vs' else 64)]
                params[0] = [f24(4)] * 4                                # n
                for i, v in enumerate(k['const_pool']): params[k['const_base'] + i] = [f24(c) for c in v]
                xs = [0.25 if l & 1 else 0.75 for l in range(16)]
                if stage == 'fs':
                    varin = [[[f24(x), 0, 0, 0]] + [[0] * 4] * 15 for x in xs]
                    res = isa64.simulate_fs2(words, isa64.FSInputs(params, varin, active=0xFFFF, interp=False))
                else:
                    inp = isa64.default_vs_inputs(rich=True)
                    inp.params = params
                    for l, x in enumerate(xs): inp.varin[l][0] = [f24(x), f24(x), f24(x), f24(x)]
                    res = isa64.simulate_vs2(words, inp)
                results.append(([real(res.outc[l][0]) for l in range(16)], res.err))
        except Exception as e:
            bad += 1
            print('FAIL %s: %s' % (name, str(e)[:110]))
            continue
        exp = [float(want[0] if x > 0.5 else want[1]) for x in xs]
        for tag, (got, err) in zip(('forward', 'plain'), results):
            if got != exp or err:
                bad += 1
                print('FAIL %s (%s): got %s, expected %s%s' % (name, tag, got[:4], exp[:4], ', core error' if err else ''))
    print('%d cases, %d failures' % (len(CASES), bad))
    return 1 if bad else 0


sys.exit(main())
