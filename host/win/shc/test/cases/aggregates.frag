// matrices and structs as whole values: assigned under a lane mask, copied without sharing, passed by value, chosen by ?:
precision mediump float;
varying vec4 v0;
uniform mat2 u_m;
struct S { vec4 a; float b; };
struct T { mat2 m; S s; float c; };
float sum(mat2 q) { q[0] = q[1] * 2.0; return q[0].x + q[1].y; }
float total(T t) { t.m[0] = vec2(1.0); t.s.b = 3.0; return t.m[0].y + t.s.b + t.c; }
void main()
{
    mat2 m = u_m;
    mat2 k = m;
    k[0] = vec2(2.0);
    if (v0.x > 0.5) m = k;
    m = m * k;
    S a = S(vec4(1.0), 1.0);
    S b = a;
    b.a.x = 2.0;
    if (v0.y > 0.5) a = b;
    T t = T(m, a, 1.0);
    T r = t;
    r.m[1] = vec2(3.0);
    if (v0.z > 0.5) { t.m = k; t.s = b; }
    if (v0.w > 0.5) r = t;
    mat2 c = v0.x > 0.25 ? m : k;
    S d = v0.y > 0.25 ? a : b;
    float z = sum(m);
    gl_FragColor = vec4(z + c[0].x + m[0].y, total(t) + total(r) + t.m[1].x, d.a.x + d.b, m[1].y);
}
