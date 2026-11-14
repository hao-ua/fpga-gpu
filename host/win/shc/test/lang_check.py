#!/usr/bin/env python3
"""lang_check — the language against the truth: every program below is compiled as an application's is (strict) and runs in the core simulator on a warp of lanes x = 0.75 and x = 0.25; all four components of the result are compared with the answer worked out by hand. flow_check does this for control flow, this one for GLSL ES 1.00's types, operators, constructors and built-ins. The second half is programs the language forbids: each must fail to compile.
    python3 -I host/win/shc/test/lang_check.py      (run from the repository root) """
import sys
sys.path.insert(0, 'tools')
import glslc, isa64, float_ref as fr


def f24(x): return fr.encode_real(isa64.F(x).limit_denominator(1 << 14), isa64.FMT)
def real(b): return float(fr.to_real(b, isa64.FMT))


FS = 'precision mediump float; varying vec4 v0; uniform float n;\n'
VS = 'attribute vec4 a0; uniform float n;\n'
A = 'mat2 a = mat2(1.0, 2.0, 3.0, 4.0); mat2 b = mat2(5.0, 6.0, 7.0, 8.0); '
P = 'mat2 p = mat2(1.0, 2.0, 4.0, 8.0); '
K = 'int k = int(v0.x * 4.0); '                                        # 3 and 1
# name, program (a main that ends by writing gl_FragColor), the result where x > 0.5, the result elsewhere (None: the same)
C = [
    ('mat + mat', A + 'mat2 c = a + b; gl_FragColor = vec4(c[0], c[1]);', (6, 8, 10, 12)),
    ('mat - mat', A + 'mat2 c = b - a; gl_FragColor = vec4(c[0], c[1]);', (4, 4, 4, 4)),
    ('mat * float', A + 'mat2 c = a * 2.0; gl_FragColor = vec4(c[0], c[1]);', (2, 4, 6, 8)),
    ('float * mat', A + 'mat2 c = 3.0 * a; gl_FragColor = vec4(c[0], c[1]);', (3, 6, 9, 12)),
    ('mat / float', A + 'mat2 c = a / 2.0; gl_FragColor = vec4(c[0], c[1]);', (.5, 1, 1.5, 2)),
    ('mat / mat', P + 'mat2 c = mat2(2.0, 2.0, 2.0, 2.0) / p; gl_FragColor = vec4(c[0], c[1]);', (2, 1, .5, .25)),
    ('float / mat', P + 'mat2 c = 8.0 / p; gl_FragColor = vec4(c[0], c[1]);', (8, 4, 2, 1)),
    ('mat + float', A + 'mat2 c = a + 1.0; gl_FragColor = vec4(c[0], c[1]);', (2, 3, 4, 5)),
    ('float - mat', A + 'mat2 c = 1.0 - a; gl_FragColor = vec4(c[0], c[1]);', (0, -1, -2, -3)),
    ('- mat', A + 'mat2 c = -a; gl_FragColor = vec4(c[0], c[1]);', (-1, -2, -3, -4)),
    ('mat * 0', A + 'mat2 c = a * 0.0; gl_FragColor = vec4(c[0], c[1]);', (0, 0, 0, 0)),
    ('mat++ gives the old', A + 'mat2 m = a; mat2 o = m++; gl_FragColor = vec4(o[0][0], m[0][0], o[1][1], m[1][1]);', (1, 2, 4, 5)),
    ('++mat', A + 'mat2 m = a; mat2 o = ++m; gl_FragColor = vec4(o[0][0], m[0][0], o[1][1], m[1][1]);', (2, 2, 5, 5)),
    ('mat * mat', A + 'mat2 c = a * b; gl_FragColor = vec4(c[0], c[1]);', (23, 34, 31, 46)),
    ('mat *= mat', A + 'mat2 c = a; c *= b; gl_FragColor = vec4(c[0], c[1]);', (23, 34, 31, 46)),
    ('mat * vec', A + 'gl_FragColor = vec4(a * vec2(1.0, 1.0), 0.0, 0.0);', (4, 6, 0, 0)),
    ('vec * mat', A + 'gl_FragColor = vec4(vec2(1.0, 1.0) * a, 0.0, 0.0);', (3, 7, 0, 0)),
    ('vec *= mat', A + 'vec2 v = vec2(1.0, 1.0); v *= a; gl_FragColor = vec4(v, 0.0, 0.0);', (3, 7, 0, 0)),
    ('mat == mat', A + 'gl_FragColor = vec4(a == a ? 1.0 : 0.0, a == b ? 1.0 : 0.0, a != a ? 1.0 : 0.0, a != b ? 1.0 : 0.0);', (1, 0, 0, 1)),
    ('mat3 of mat2', 'mat3 m = mat3(mat2(1.0, 2.0, 3.0, 4.0)); gl_FragColor = vec4(m[0][1], m[1][0], m[2][2], m[2][0]);', (2, 3, 1, 0)),
    ('mat2 of mat3', 'mat2 m = mat2(mat3(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0)); gl_FragColor = vec4(m[0], m[1]);', (1, 2, 4, 5)),
    ('mat4 of mat2', 'mat4 m = mat4(mat2(1.0, 2.0, 3.0, 4.0)); gl_FragColor = vec4(m[1][1], m[2][2], m[3][3], m[3][0]);', (4, 1, 1, 0)),
    ('mat2 of vec3, float', 'mat2 m = mat2(vec3(1.0, 2.0, 3.0), 4.0); gl_FragColor = vec4(m[0], m[1]);', (1, 2, 3, 4)),
    ('mat2 of vec4', 'mat2 m = mat2(vec4(1.0, 2.0, 3.0, 4.0)); gl_FragColor = vec4(m[0], m[1]);', (1, 2, 3, 4)),
    ('mat3 of vec2s', 'vec2 w = vec2(7.0, 8.0); mat3 m = mat3(w, 1.0, w, 2.0, w, 3.0); gl_FragColor = vec4(m[0][2], m[1][2], m[2][2], m[2][1]);', (1, 2, 3, 8)),
    ('vec4 of mat2', 'gl_FragColor = vec4(mat2(1.0, 2.0, 3.0, 4.0));', (1, 2, 3, 4)),
    ('vec3 of mat2', 'gl_FragColor = vec4(vec3(mat2(1.0, 2.0, 3.0, 4.0)), 9.0);', (1, 2, 3, 9)),
    ('a column assigned', 'mat3 m = mat3(1.0); m[1] = vec3(5.0, 6.0, 7.0); gl_FragColor = vec4(m[1], m[0][0]);', (5, 6, 7, 1)),
    ('a matrix element assigned', 'mat3 m = mat3(1.0); m[2][1] = 3.0; m[0].yz = vec2(8.0, 9.0); gl_FragColor = vec4(m[2][1], m[0]);', (3, 1, 8, 9)),
    ('float of a vector', 'gl_FragColor = vec4(float(vec3(2.0, 3.0, 4.0)), float(int(vec2(3.5, 1.0))), float(bool(vec2(0.0, 1.0))), float(mat2(5.0)));', (2, 3, 0, 5)),
    # ints
    ('int * and / in order', 'int i = int(n); gl_FragColor = vec4(float(5 * i / 2), float(7 * i / 2 * 2), float((i + 3) / 2 * 2), float(-7 * i / 8));', (10, 28, 6, -3)),
    ('int ==', 'int i = int(n); gl_FragColor = vec4(i == 4 ? 1.0 : 0.0, i != 4 ? 1.0 : 0.0, 4 == i ? 1.0 : 0.0, i + 1 == 4 ? 1.0 : 0.0);', (1, 0, 1, 0)),
    ('const int /', 'const int c = 7 / 2; const int d = -7 / 2; const int e = 2 * 3 + 1; gl_FragColor = vec4(float(c), float(d), float(e), 0.0);', (3, -3, 7, 0)),
    ('tracked int /', 'int p = 7; int q = p / 2; int r = q; r /= 2; gl_FragColor = vec4(float(q), float(r), 0.0, 0.0);', (3, 1, 0, 0)),
    ('ivec of floats', 'ivec2 w = ivec2(vec2(1.75, -1.75)); ivec3 u = ivec3(2.5); gl_FragColor = vec4(float(w.x), float(w.y), float(u.z), 0.0);', (1, -1, 2, 0)),
    ('ivec /', 'ivec2 w = ivec2(7, -7) / ivec2(2, 2); ivec2 u = ivec2(9, 8) / 4; gl_FragColor = vec4(vec2(w), vec2(u));', (3, -3, 2, 2)),
    ('ivec / exact quotient', 'int t = int(n) + 2; ivec2 w = ivec2(t * 7, t * 3) / ivec2(7, 3); gl_FragColor = vec4(vec2(w), 0.0, 0.0);', (6, 6, 0, 0)),
    ('bvec of floats', 'gl_FragColor = vec4(vec3(bvec3(2.0, 0.0, -1.0)), 7.0);', (1, 0, 1, 7)),
    ('bvec of ints', 'gl_FragColor = vec4(vec2(bvec2(ivec2(3, 0))), vec2(bvec2(1, 0)));', (1, 0, 1, 0)),
    ('bool of a float', 'gl_FragColor = vec4(float(bool(v0.x - 0.25)), float(bool(1)), float(int(true)), float(false));', (1, 1, 1, 0), (0, 1, 1, 0)),
    ('an int in a struct', 'struct I { int i; float f; }; I s; s.i = 3; s.f = 1.0; I t = I(5, 2.0); I u = t; gl_FragColor = vec4(float(s.i + 1), float(t.i + 1), float(u.i * 2), 0.0);', (4, 6, 10, 0)),
    ('an int in an array', 'int a[2]; a[1] = 5; a[0] = a[1] + 1; gl_FragColor = vec4(float(a[0]), float(a[1]), 0.0, 0.0);', (6, 5, 0, 0)),
    # functions, built-ins of one component
    ('dot of floats', 'gl_FragColor = vec4(dot(2.0, 3.0), length(-2.0), distance(1.0, 4.0), normalize(-3.0));', (6, 2, 3, -1)),
    ('mod with a vector', 'gl_FragColor = vec4(mod(vec2(5.0, 7.0), vec2(3.0, 4.0)), mod(vec2(5.0, 7.0), 3.0));', (2, 3, 2, 1)),
    ('smoothstep with vector edges', 'gl_FragColor = vec4(smoothstep(vec2(0.0, 0.0), vec2(2.0, 4.0), vec2(1.0, 2.0)), smoothstep(vec2(0.0, 4.0), vec2(1.0, 8.0), vec2(0.0, 8.0)));', (.5, .5, 0, 1)),
    ('x++ gives the old', 'float i = v0.x; float o = i++; gl_FragColor = vec4(o, i, 0.0, 0.0);', (.75, 1.75, 0, 0), (.25, 1.25, 0, 0)),
    ('++x gives the new', 'float i = v0.x; float o = ++i; gl_FragColor = vec4(o, i, 0.0, 0.0);', (1.75, 1.75, 0, 0), (1.25, 1.25, 0, 0)),
    ('?: runs one side', 'float a = 1.0; float r = v0.x > 0.5 ? (a = 2.0) : 3.0; gl_FragColor = vec4(r, a, 0.0, 0.0);', (2, 2, 0, 0), (3, 1, 0, 0)),
    ('&& runs its right only where the left holds', 'float a = 1.0; bool b = v0.x > 0.5 && (a = 2.0) > 0.0; gl_FragColor = vec4(a, b ? 1.0 : 0.0, 0.0, 0.0);', (2, 1, 0, 0), (1, 0, 0, 0)),
    ('|| runs its right only where the left fails', 'float a = 1.0; bool b = v0.x > 0.5 || (a = 2.0) > 0.0; gl_FragColor = vec4(a, b ? 1.0 : 0.0, 0.0, 0.0);', (1, 1, 0, 0), (2, 1, 0, 0)),
    ('swizzles stpq rgba', 'vec4 w = vec4(1.0, 2.0, 3.0, 4.0); w.st += vec2(10.0, 20.0); w.pq -= vec2(1.0, 1.0); gl_FragColor = w.abgr;', (3, 2, 22, 11)),
    ('swizzle of a swizzle', 'vec4 w = vec4(1.0, 2.0, 3.0, 4.0); w.wz.y = 9.0; gl_FragColor = vec4(w.xy.yx.xy, w.zw);', (2, 1, 9, 4)),
    ('a vector component assigned', 'vec4 c = vec4(0.0); c[2] = 3.0; for (int i = 0; i < 2; i++) c[i] = float(i + 1); gl_FragColor = c;', (1, 2, 3, 0)),
    ('an element as an out argument', 'void f(out float x) { x = 5.0; }\nvoid main() { vec2 c = vec2(1.0); f(c[1]); gl_FragColor = vec4(c, 0.0, 0.0); }\n//', (1, 5, 0, 0)),
    ('run-time index of a vector', K + 'vec4 w = vec4(10.0, 20.0, 30.0, 40.0); gl_FragColor = vec4(w[k]);', (40, 40, 40, 40), (20, 20, 20, 20)),
    ('run-time index of a matrix', K + 'mat4 m = mat4(1.0); m[3][0] = 5.0; gl_FragColor = m[k];', (5, 0, 0, 1), (0, 1, 0, 0)),
    ('run-time index of a local array', K + 'float a[4]; for (int i = 0; i < 4; i++) a[i] = float(i * 2); gl_FragColor = vec4(a[k]);', (6, 6, 6, 6), (2, 2, 2, 2)),
    ('a vector written at a run-time index', K + 'vec4 c = vec4(0.0); c[k] = 1.0; gl_FragColor = c;', (0, 0, 0, 1), (0, 1, 0, 0)),
    ('an array written at a run-time index', K + 'float a[4]; for (int i = 0; i < 4; i++) a[i] = 0.0; a[k] = 1.0; gl_FragColor = vec4(a[0], a[1], a[2], a[3]);', (0, 0, 0, 1), (0, 1, 0, 0)),
    ('loop on a local bound', 'int m = int(n); int i = 0; float s = 0.0; while (i < m) { s += 1.0; i++; } gl_FragColor = vec4(s);', (4, 4, 4, 4)),
    ('while on a constant', 'int i = 0; float s = 0.0; while (i < 3) { s += 1.0; i++; } gl_FragColor = vec4(s);', (3, 3, 3, 3)),
    ('do while', 'int i = 0; do { i++; } while (i < 3); gl_FragColor = vec4(float(i));', (3, 3, 3, 3)),
    ('for with no condition', 'float s = 0.0; for (int i = 0; ; i++) { if (i >= 2) break; s += 1.0; } gl_FragColor = vec4(s);', (2, 2, 2, 2)),
    ('for with no step', 'float s = 0.0; for (int i = 0; i < 3; ) { s += 1.0; i++; } gl_FragColor = vec4(s);', (3, 3, 3, 3)),
    ('const hides nothing of a local', 'vec2 c = vec2(2.0); gl_FragColor = vec4(c, K, K);', (2, 2, 3, 3)),
    ('literals', 'gl_FragColor = vec4(float(0x10), float(010), float(0xFF), 1e1);', (16, 8, 255, 10)),
    ('built-in constants', 'gl_FragColor = vec4(float(gl_MaxVaryingVectors), float(gl_MaxVertexAttribs), float(gl_MaxDrawBuffers), float(gl_MaxTextureImageUnits));', (8, 16, 1, 8)),
    ('gl_FragData[constant]', 'const int z = 0; gl_FragData[z] = vec4(1.0, 2.0, 3.0, 4.0);', (1, 2, 3, 4)),
    ('gl_FragData[loop]', 'for (int i = 0; i < gl_MaxDrawBuffers; i++) gl_FragData[i] = vec4(1.0, 2.0, 3.0, 4.0);', (1, 2, 3, 4)),
    ('gl_FragData in part', 'const int z = 0; gl_FragData[z] = vec4(1.0, 2.0, 3.0, 4.0); gl_FragData[z].y = 7.0;', (1, 7, 3, 4)),
]
# whole programs: name, source after FS, result
W = [
    ('overloads', 'float f(float a) { return a + 1.0; }\nvec2 f(vec2 a) { return a + 2.0; }\nint f(int a) { return a + 3; }\nvoid main() { gl_FragColor = vec4(f(1.0), f(vec2(1.0)), float(f(1))); }\n', (2, 3, 3, 4)),
    ('an overload in an if', 'void f(out float a) { a = 1.0; }\nvoid f(out vec2 a) { a = vec2(2.0); }\nvoid main() { float x = 0.0; vec2 y = vec2(0.0); if (v0.x > 0.5) f(x); else f(y); gl_FragColor = vec4(x, y, 0.0); }\n', (1, 0, 0, 0), (0, 2, 2, 0)),
    ('a prototype', 'float g(float);\nfloat h(float a, float);\nvoid main() { gl_FragColor = vec4(g(1.0), h(1.0, 2.0), 0.0, 0.0); }\nfloat g(float a) { return a * 2.0; }\nfloat h(float a, float b) { return a + b; }\n', (2, 3, 0, 0)),
    ('struct ==', 'struct S { vec4 a; float b; mat2 m; };\nvoid main() { S x = S(vec4(1.0), 2.0, mat2(1.0)); S y = S(vec4(1.0), 2.0, mat2(1.0)); S z = S(vec4(1.0), 3.0, mat2(1.0)); gl_FragColor = vec4(x == y ? 1.0 : 0.0, x == z ? 1.0 : 0.0, x != z ? 1.0 : 0.0, 0.0); }\n', (1, 0, 1, 0)),
    ('a struct defined in a block hides', 'struct S { float a; };\nvoid main() { S o = S(1.0); float r = 0.0; { struct S { float b; float c; }; S t = S(2.0, 3.0); r = t.c; } gl_FragColor = vec4(o.a, r, 0.0, 0.0); }\n', (1, 3, 0, 0)),
    ('struct and variable at once', 'struct S { vec4 c; } g;\nvoid main() { g.c = vec4(2.0); gl_FragColor = g.c; }\n', (2, 2, 2, 2)),
    ('anonymous struct', 'struct { vec2 p; } g;\nvoid main() { g.p = vec2(3.0); struct { float q; } l; l.q = 1.0; gl_FragColor = vec4(g.p, l.q, 0.0); }\n', (3, 3, 1, 0)),
    ('uniform struct at once', 'uniform struct U { float k; } u;\nvoid main() { gl_FragColor = vec4(u.k); }\n', (0, 0, 0, 0)),
    ('const of a function of a const', 'const float s = sin(0.0) + cos(0.0);\nconst int m = int(s) + 1;\nfloat a[m > 1 ? 2 : 1];\nvoid main() { a[1] = s; gl_FragColor = vec4(a[1], float(m), 0.0, 0.0); }\n', (1, 2, 0, 0)),
    ('two consts on a line', 'const float x = 1.0, y = 2.0;\nvoid main() { gl_FragColor = vec4(x, y, 0.0, 0.0); }\n', (1, 2, 0, 0)),
    ('global const, a caller\'s local of the name', 'const vec2 c = vec2(1.0);\nvec2 f() { return c; }\nvoid main() { vec2 c = vec2(2.0); gl_FragColor = vec4(c, f()); }\n', (2, 2, 1, 1)),
    ('global const, its initializer sees globals', 'const float k = 3.0; const vec2 c = vec2(k);\nvoid main() { float k = 1.0; gl_FragColor = vec4(c, k, 0.0); }\n', (3, 3, 1, 0)),
    ('precision in a function', 'void main() { precision lowp float; lowp vec4 c = vec4(1.0); highp float h = 2.0; mediump int i = 3; const lowp float k = 4.0; gl_FragColor = c * h + float(i) + k; }\n', (9, 9, 9, 9)),
    ('a constructor starts a statement', 'void main() { float(1.0) + 2.0; vec2(1.0).x; gl_FragColor = vec4(1.0); }\n', (1, 1, 1, 1)),
    ('a uniform array of ints', 'uniform int a[2];\nvoid main() { gl_FragColor = vec4(float(a[1] + 1)); }\n', (1, 1, 1, 1)),
    ('#line', '#line 40\nvoid main() { gl_FragColor = vec4(float(__LINE__)); }\n', (41, 41, 41, 41)),
]
# (name, stage, source): each must not compile
B = 'precision mediump float; uniform vec4 u; uniform float f; uniform int ii; varying vec2 w;\n'
X3 = '#extension GL_OES_texture_3D : enable\n'
NO = [
    ('#version 300 es', 'fs', '#version 300 es\nvoid main() { }\n'),
    ('#version after code', 'fs', 'precision mediump float;\n#version 100\nvoid main() { }\n'),
    ('#extension require unknown', 'fs', '#extension GL_FOO : require\nvoid main() { }\n'),
    ('#if with no #endif', 'fs', '#if 1\nvoid main() { }\n'),
    ('#define GL_', 'fs', '#define GL_FOO 1\nvoid main() { }\n'),
    ('no main', 'fs', B),
    ('main with a type', 'fs', B + 'float main() { return 1.0; }\n'),
    ('main with an argument', 'fs', B + 'void main(float a) { }\n'),
    ('a reserved word', 'fs', B + 'void main() { int goto = 1; }\n'),
    ('switch', 'fs', B + 'void main() { switch (1) { } }\n'),
    ('gl_ name', 'fs', B + 'float gl_x;\nvoid main() { }\n'),
    ('int + float', 'fs', B + 'void main() { float a = 1.0 + 1; }\n'),
    ('float = int', 'fs', B + 'void main() { float a = 1; }\n'),
    ('int = float', 'fs', B + 'void main() { int a = 1.0; }\n'),
    ('vec3 + vec2', 'fs', B + 'void main() { vec3 a = vec3(1.0) + vec2(1.0); }\n'),
    ('mat3 * vec2', 'fs', B + 'void main() { vec2 a = mat3(1.0) * vec2(1.0); }\n'),
    ('ivec * vec', 'fs', B + 'void main() { ivec2 a = ivec2(1) * vec2(1.0); }\n'),
    ('%', 'fs', B + 'void main() { int a = 5 % 2; }\n'),
    ('<<', 'fs', B + 'void main() { int a = 5 << 2; }\n'),
    ('bitwise &', 'fs', B + 'void main() { int a = 5 & 2; }\n'),
    ('! of an int', 'fs', B + 'void main() { bool a = !1; }\n'),
    ('&& of ints', 'fs', B + 'void main() { bool a = 1 && 0; }\n'),
    ('if of a float', 'fs', B + 'void main() { if (1.0) { } }\n'),
    ('while of an int', 'fs', B + 'void main() { while (1) { } }\n'),
    ('< of vectors', 'fs', B + 'void main() { bool a = vec2(1.0) < vec2(2.0); }\n'),
    ('?: of two types', 'fs', B + 'void main() { float a = true ? 1.0 : 1; }\n'),
    ('?: on a float', 'fs', B + 'void main() { float a = 1.0 ? 1.0 : 2.0; }\n'),
    ('bad swizzle letters', 'fs', B + 'void main() { vec2 a = u.xg; }\n'),
    ('swizzle out of range', 'fs', B + 'void main() { float a = w.z; }\n'),
    ('five letters', 'fs', B + 'void main() { vec4 a = u.xyzwx; }\n'),
    ('swizzle of a float', 'fs', B + 'void main() { float a = f.x; }\n'),
    ('write through v.xx', 'fs', B + 'void main() { vec2 a; a.xx = vec2(1.0); }\n'),
    ('index out of range', 'fs', B + 'void main() { vec2 a; a[2] = 1.0; }\n'),
    ('index of an array out of range', 'fs', B + 'void main() { float a[2]; a[2] = 1.0; }\n'),
    ('float index', 'fs', B + 'void main() { vec2 a; a[0.0] = 1.0; }\n'),
    ('negative array size', 'fs', B + 'void main() { float a[-1]; }\n'),
    ('array size not constant', 'fs', B + 'void main() { float a[ii]; }\n'),
    ('array initializer', 'fs', B + 'void main() { float a[2] = 1.0; }\n'),
    ('undefined variable', 'fs', B + 'void main() { float a = b; }\n'),
    ('undefined function', 'fs', B + 'void main() { foo(1.0); }\n'),
    ('wrong argument type', 'fs', B + 'float g(float a) { return a; }\nvoid main() { g(1); }\n'),
    ('wrong argument count', 'fs', B + 'float g(float a) { return a; }\nvoid main() { g(1.0, 2.0); }\n'),
    ('wrong type to a built-in', 'fs', B + 'void main() { float a = sin(1); }\n'),
    ('dot of two sizes', 'fs', B + 'void main() { float a = dot(vec2(1.0), vec3(1.0)); }\n'),
    ('return of a wrong type', 'fs', B + 'float g() { return vec2(1.0); }\nvoid main() { }\n'),
    ('return of nothing', 'fs', B + 'float g() { return; }\nvoid main() { }\n'),
    ('return of a value from void', 'fs', B + 'void g() { return 1.0; }\nvoid main() { }\n'),
    ('redefined function', 'fs', B + 'void g() { }\nvoid g() { }\nvoid main() { }\n'),
    ('two return types', 'fs', B + 'float g();\nint g();\nvoid main() { }\n'),
    ('recursion', 'fs', B + 'float g(float a) { return g(a); }\nvoid main() { }\n'),
    ('mutual recursion', 'fs', B + 'float h(float a);\nfloat g(float a) { return h(a); }\nfloat h(float a) { return g(a); }\nvoid main() { }\n'),
    ('redeclared in a scope', 'fs', B + 'void main() { float a; int a; }\n'),
    ('assign to a const', 'fs', B + 'const float k = 1.0;\nvoid main() { k = 2.0; }\n'),
    ('assign to a uniform', 'fs', B + 'void main() { f = 2.0; }\n'),
    ('assign to a varying in a fragment shader', 'fs', B + 'void main() { w = vec2(1.0); }\n'),
    ('assign to an attribute', 'vs', 'attribute vec4 a;\nvoid main() { a = vec4(1.0); }\n'),
    ('assign to a constant expression', 'fs', B + 'void main() { 1.0 = 2.0; }\n'),
    ('assign to a call', 'fs', B + 'void main() { vec2(1.0) = vec2(2.0); }\n'),
    ('assign to a const parameter', 'fs', B + 'void g(const float a) { a = 1.0; }\nvoid main() { }\n'),
    ('out argument that cannot be written', 'fs', B + 'void g(out float a) { a = 1.0; }\nvoid main() { g(f); }\n'),
    ('out argument: a literal', 'fs', B + 'void g(inout float a) { a = 1.0; }\nvoid main() { g(1.0); }\n'),
    ('assign to gl_FragCoord', 'fs', B + 'void main() { gl_FragCoord = vec4(1.0); }\n'),
    ('gl_FragColor in a vertex shader', 'vs', 'void main() { gl_FragColor = vec4(1.0); }\n'),
    ('gl_Position in a fragment shader', 'fs', B + 'void main() { gl_Position = vec4(1.0); }\n'),
    ('gl_FragDepth is not in ES 1.00', 'fs', B + 'void main() { gl_FragDepth = 1.0; }\n'),
    ('gl_FragData[1]', 'fs', B + 'void main() { gl_FragData[1] = vec4(1.0); }\n'),
    ('attribute in a fragment shader', 'fs', B + 'attribute vec4 a;\nvoid main() { }\n'),
    ('attribute of a struct', 'vs', 'struct S { vec4 a; };\nattribute S s;\nvoid main() { }\n'),
    ('varying of an int', 'vs', 'varying int v;\nvoid main() { }\n'),
    ('a uniform with an initializer', 'fs', B + 'uniform float q = 1.0;\nvoid main() { }\n'),
    ('const without an initializer', 'fs', B + 'const float q;\nvoid main() { }\n'),
    ('const of a variable', 'fs', B + 'void main() { const float q = f; }\n'),
    ('global initialized by a uniform', 'fs', B + 'float g = f;\nvoid main() { }\n'),
    ('a local sampler', 'fs', B + 'void main() { sampler2D s; }\n'),
    ('a void variable', 'fs', B + 'void main() { void a; }\n'),
    ('no precision for float', 'fs', 'void main() { float a = 1.0; }\n'),
    ('no precision for a uniform', 'fs', 'uniform vec4 a;\nvoid main() { }\n'),
    ('no precision for a struct member', 'fs', 'struct S { vec4 a; };\nvoid main() { }\n'),
    ('no precision for a parameter', 'fs', 'void g(float a) { }\nvoid main() { }\n'),
    ('no precision for a return', 'fs', 'vec4 g() { return vec4(1.0); }\nvoid main() { }\n'),
    ('precision of a vector', 'fs', 'precision mediump vec4;\nvoid main() { }\n'),
    ('discard in a vertex shader', 'vs', 'void main() { discard; }\n'),
    ('break outside a loop', 'fs', B + 'void main() { break; }\n'),
    ('continue outside a loop', 'fs', B + 'void main() { continue; }\n'),
    ('a constructor with nothing', 'fs', B + 'void main() { vec2 a = vec2(); }\n'),
    ('a constructor with too little', 'fs', B + 'void main() { vec3 a = vec3(1.0, 2.0); }\n'),
    ('a constructor with too much', 'fs', B + 'void main() { vec2 a = vec2(1.0, 2.0, 3.0); }\n'),
    ('a matrix with something else', 'fs', B + 'void main() { mat2 a = mat2(mat2(1.0), 1.0); }\n'),
    ('a struct constructor, wrong count', 'fs', B + 'struct S { float a; float b; };\nvoid main() { S s = S(1.0); }\n'),
    ('a struct constructor, wrong type', 'fs', B + 'struct S { float a; };\nvoid main() { S s = S(1); }\n'),
    ('no such member', 'fs', B + 'struct S { float a; };\nvoid main() { S s = S(1.0); s.b = 1.0; }\n'),
    ('== of arrays', 'fs', B + 'void main() { float a[2]; float b[2]; bool c = a == b; }\n'),
    ('== of a struct with an array', 'fs', B + 'struct S { float a[2]; };\nvoid main() { S x; S y; bool c = x == y; }\n'),
    ('array assignment with a sampler in a struct', 'fs', B + 'struct S { sampler2D s; };\nuniform S a; uniform S b;\nvoid main() { S c; }\n'),
    ('dFdx with no extension', 'fs', B + 'void main() { vec2 a = dFdx(w); }\n'),
    ('texture2DLod in a fragment shader', 'fs', B + 'uniform sampler2D s;\nvoid main() { vec4 c = texture2DLod(s, w, 0.0); }\n'),
    ('texture2DLodEXT with no extension', 'fs', B + 'uniform sampler2D s;\nvoid main() { vec4 c = texture2DLodEXT(s, w, 0.0); }\n'),
    ('gl_FragDepthEXT with no extension', 'fs', B + 'void main() { gl_FragDepthEXT = 1.0; }\n'),
    ('a bias in a vertex shader', 'vs', 'uniform sampler2D s; attribute vec2 a;\nvoid main() { gl_Position = texture2D(s, a, 1.0); }\n'),
    ('sampler3D with no extension', 'fs', B + 'uniform sampler3D s;\nvoid main() { }\n'),
    ('sampler3D after disable', 'fs', '#extension GL_OES_texture_3D : enable\n#extension GL_OES_texture_3D : disable\n' + B + 'uniform sampler3D s;\nvoid main() { }\n'),
    ('texture3D with no extension', 'fs', B + 'uniform sampler2D s;\nvoid main() { vec4 c = texture3D(s, vec3(w, 0.0)); }\n'),
    ('texture3D of a sampler2D', 'fs', X3 + B + 'uniform sampler2D s;\nvoid main() { vec4 c = texture3D(s, vec3(w, 0.0)); }\n'),
    ('texture3D of a vec2', 'fs', X3 + B + 'uniform sampler3D s;\nvoid main() { vec4 c = texture3D(s, w); }\n'),
    ('texture3DProj of a vec3', 'fs', X3 + B + 'uniform sampler3D s;\nvoid main() { vec4 c = texture3DProj(s, vec3(w, 1.0)); }\n'),
    ('texture2D of a sampler3D', 'fs', X3 + B + 'uniform sampler3D s;\nvoid main() { vec4 c = texture2D(s, w); }\n'),
    ('texture3DLod in a fragment shader', 'fs', X3 + B + 'uniform sampler3D s;\nvoid main() { vec4 c = texture3DLod(s, vec3(w, 0.0), 0.0); }\n'),
    ('texture3DProjLod in a fragment shader', 'fs', X3 + B + 'uniform sampler3D s;\nvoid main() { vec4 c = texture3DProjLod(s, vec4(w, 0.0, 1.0), 0.0); }\n'),
    ('a bias on texture3D in a vertex shader', 'vs', X3 + 'uniform sampler3D s; attribute vec3 a;\nvoid main() { gl_Position = texture3D(s, a, 1.0); }\n'),
    ('a local sampler3D', 'fs', X3 + B + 'void main() { sampler3D s; }\n'),
    ('an out sampler3D', 'fs', X3 + B + 'void f(out sampler3D s) { }\nvoid main() { }\n'),
]


def run(src, stage, strict=True):
    res = []
    for fwd in (True, False):
        k = glslc.compile_stage(src, stage, isa64=True, forward=fwd, strict=strict)
        words, _ = isa64.assemble(k['sasm'].splitlines(), vs=stage == 'vs')
        params = [[0, 0, 0, 0] for _ in range(256 if stage == 'vs' else 64)]
        params[0] = [f24(4)] * 4                                        # n
        for i, v in enumerate(k['const_pool']): params[k['const_base'] + i] = [f24(c) for c in v]
        xs = [0.25 if l & 1 else 0.75 for l in range(16)]
        if stage == 'fs':
            varin = [[[f24(x), 0, 0, 0]] + [[0] * 4] * 15 for x in xs]
            r = isa64.simulate_fs2(words, isa64.FSInputs(params, varin, active=0xFFFF, interp=False))
        else:
            inp = isa64.default_vs_inputs(rich=True)
            inp.params = params
            for l, x in enumerate(xs): inp.varin[l][0] = [f24(x), f24(x), f24(x), f24(x)]
            r = isa64.simulate_vs2(words, inp)
        res.append(([[real(c) for c in r.outc[l]] for l in range(16)], r.err))
    return res, xs


def check(name, src, stage, hi, lo):
    try:
        res, xs = run(src, stage)
    except Exception as e:
        print('FAIL %s: %s' % (name, str(e)[:140]))
        return 1
    bad = 0
    for tag, (got, err) in zip(('forward', 'plain'), res):
        for l, x in enumerate(xs):
            want = [float(c) for c in (hi if x > 0.5 else lo)]
            if any(abs(a - b) > 1e-3 * max(1.0, abs(b)) for a, b in zip(got[l], want)) or err:
                print('FAIL %s (%s): lane %d got %s, expected %s%s' % (name, tag, l, got[l], want, ', core error' if err else ''))
                bad = 1
                break
    return bad


def dump(d):
    """every program as a file: compare.ps1 holds the C compiler to these too"""
    import os, re
    os.makedirs(d, exist_ok=True)
    def name(s): return re.sub(r'[^a-z0-9]+', '_', s.lower()).strip('_')
    for i, c in enumerate(C):
        pre = FS + ('const float K = 3.0;\n' if 'K)' in c[1] else '')
        open('%s/v%03d_%s.frag' % (d, i, name(c[0])), 'w', newline='').write(pre + (c[1] if c[1].endswith('//') else 'void main() { ' + c[1] + ' }\n'))
    for i, c in enumerate(W):
        open('%s/w%03d_%s.frag' % (d, i, name(c[0])), 'w', newline='').write(FS + c[1])
    for i, (nm, st, src) in enumerate(NO):
        open('%s/no%03d_%s.%s' % (d, i, name(nm), 'vert' if st == 'vs' else 'frag'), 'w', newline='').write(src)


def main():
    if len(sys.argv) > 2 and sys.argv[1] == '--dump':
        dump(sys.argv[2])
        return 0
    bad = 0
    for c in C:
        name, body, hi = c[0], c[1], c[2]
        lo = c[3] if len(c) > 3 else hi
        pre = FS + ('const float K = 3.0;\n' if 'K)' in body else '')
        src = body if body.endswith('//') else 'void main() { ' + body + ' }\n'
        bad += check(name, pre + src, 'fs', hi, lo)
    for c in W:
        name, body, hi = c[0], c[1], c[2]
        lo = c[3] if len(c) > 3 else hi
        pre = FS if not body.startswith('#') else FS
        bad += check(name, pre + body, 'fs', hi, lo)
    vs = [
        ('VS: uniform vector at a run-time index', VS + 'uniform vec4 w;\nvoid main() { gl_Position = vec4(w[int(a0.x * 4.0)] + 1.0); }\n', (1, 1, 1, 1)),
        ('VS: uniform matrix at a run-time index', VS + 'uniform mat4 m;\nvoid main() { gl_Position = m[int(a0.x * 4.0)] + 1.0; }\n', (1, 1, 1, 1)),
        ('VS: a local vector at a run-time index', VS + 'void main() { vec4 q = vec4(1.0, 2.0, 3.0, 4.0); gl_Position = vec4(q[int(a0.x * 4.0)]); }\n', (4, 4, 4, 4), (2, 2, 2, 2)),
        ('VS: a sampler compiles, the link refuses', VS + 'uniform sampler2D s;\nvoid main() { gl_Position = texture2DLod(s, a0.xy, 0.0) + 1.0; }\n', (1, 1, 1, 1)),
    ]
    for c in vs:
        bad += check(c[0], c[1], 'vs', c[2], c[3] if len(c) > 3 else c[2])
    if not glslc.compile_stage(vs[3][1], 'vs', isa64=True, strict=True)['vs_tex']:
        print('FAIL vs_tex not set'); bad += 1
    no = 0
    for name, stage, src in NO:
        try:
            glslc.compile_stage(src, stage, isa64=True, forward=True, strict=True)
            print('FAIL accepted: %s' % name)
            no += 1
        except glslc.CompileError:
            pass
        except Exception as e:
            print('FAIL %s: not a compile error: %s %s' % (name, type(e).__name__, str(e)[:100]))
            no += 1
    print('%d programs, %d failures; %d refused, %d not' % (len(C) + len(W) + len(vs), bad, len(NO), no))
    return 1 if bad or no else 0


sys.exit(main())
