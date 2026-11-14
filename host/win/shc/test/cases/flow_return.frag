// a return in some lanes only: under if, past calls, in loops of both kinds, with one result per lane. host/win/shc/test/flow_check.py has the answers
precision mediump float;
varying vec4 v0;
uniform float n;
struct S { vec4 a; float b; };
float first(float x) { if (x > 0.9) return 9.0; if (x > 0.5) return 1.0; if (x > 0.1) return 2.0; return 8.0; }
float inner(float x) { if (x > 0.5) return first(x); else return 2.0; }
float loops(float x)
{
    float s = 0.0;
    for (float i = 0.0; i < n; i += 1.0) {
        for (float j = 0.0; j < n; j += 1.0) {
            if (x > 0.5) return 1.0;
        }
        s = 2.0;
    }
    return s;
}
float rolled(float x)
{
    float s = 2.0;
    for (int q = 0; q < 3; q++) {
        if (x > 0.5) return 1.0;
        s = 2.0;
    }
    return s;
}
float looped(float x)
{
    float k = 0.0;
    do {
        if (x > 0.5) return k;
        k += 1.0;
    } while (k < n);
    return k;
}
mat2 pick(float x)
{
    if (x > 0.5) return mat2(1.0, 1.0, 1.0, 1.0);
    return mat2(2.0, 2.0, 2.0, 2.0);
}
S choose(float x)
{
    S a = S(vec4(1.0), 1.0);
    S b = S(vec4(2.0), 2.0);
    if (x > 0.5) return a;
    return b;
}
void paint(out vec4 c, float x)
{
    if (x > 0.5) {
        c = vec4(1.0);
        return;
    }
    c = vec4(2.0);
}
void main()
{
    vec4 c;
    paint(c, v0.x);
    mat2 m = pick(v0.y);
    S s = choose(v0.z);
    float t = 5.0;
    if (v0.y > 0.0) t = first(v0.y);
    gl_FragColor = c * first(v0.x) + inner(v0.y) * s.a + loops(v0.z) * m[0].xyxy + vec4(rolled(v0.w) + looped(v0.w) + t);
    if (v0.w > 0.75) return;
    gl_FragColor *= 2.0;
    return;
}
