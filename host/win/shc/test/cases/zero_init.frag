// zero_init.frag — a variable declared without a value starts at 0 (glslc._zero / shc_gen.c zero_val, 2026-11-13): locals of a loop
// body every pass, vectors, ints, structs, arrays, matrices, globals, out parameters (aliased, part of a vector, the same variable
// as an in argument), and the Shadertoy form `float z, d, i; for (s *= i; i++ < 4.0; )`
precision mediump float;
varying vec4 v0;
uniform float n;

struct S { float a; vec2 b; };
float g;

void addo(out float o) { o += 3.0; }
void addio(float a, out float o) { o += a; }
void part(out vec2 o) { o.x = 1.0; }

float local(float x)   { float s = 0.0; for (int k = 0; k < 3; k++) { float a; a += x > 0.5 ? 1.0 : 2.0; s += a; } return s; }
float runtime(float x) { float s = 0.0; for (float k = 0.0; k < n; k += 1.0) { float a; a += x > 0.5 ? 1.0 : 2.0; s += a; if (k > 1.5) break; } return s; }
float vec(float x)     { float s = 0.0; for (int k = 0; k < 3; k++) { vec3 p; p.x += 1.0; p.y += x > 0.5 ? 1.0 : 2.0; s += p.x + p.y + p.z; } return s; }
float integer(float x) { float s = 0.0; for (int k = 0; k < 4; k++) { int c; c += 2; s += float(c); } return s + x; }
float arms(float x)    { float s = 0.0; for (int k = 0; k < 3; k++) { if (x > 0.5) { float a; a += 2.0; s += a; } else { float b; b += 1.0; s += b * 10.0; } } return s; }
float outp(float x)    { float y = 5.0; if (x > 0.5) addo(y); vec2 v = vec2(5.0); addo(v.y); float w = 5.0; addio(w, w); vec2 q = vec2(5.0); part(q); return y + v.x + v.y + w + q.x + q.y; }
float aggr(float x)    { float s = 0.0; for (int k = 0; k < 2; k++) { S q; q.a += 1.0; q.b.y += 2.0; float a[3]; a[1] += x; mat2 m; m[1] += vec2(1.0, 2.0); s += q.a + q.b.y + q.b.x + a[0] + a[1] + a[2] + m[0].x + m[1].y; } return s; }
float toy(float x)     { float z, d, i; float s = 0.0; for (s *= i; i++ < 4.0; ) { for (d = 0.0; d++ < 2.0; ) z += x > 0.5 ? 1.0 : 2.0; s += z; } return s + i; }

void main()
{
    float x = v0.x;
    for (int k = 0; k < 3; k++) g += x > 0.5 ? 1.0 : 2.0;
    gl_FragColor = vec4(local(x) + runtime(x) + vec(x), integer(x) + arms(x) + outp(x), aggr(x) + toy(x), g);
}
