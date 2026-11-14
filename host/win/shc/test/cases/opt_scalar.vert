// scalars are packed one per component: a copy of one reads no swizzle, whatever the reader asked for, and a masked write of one is a whole write. Swizzling a scalar is not ES 1.00, but both compilers take it
attribute vec4 a_p;
attribute vec4 a_q;
uniform float k;
varying vec4 v0;
varying vec4 v1;
varying vec4 v2;
void main()
{
    float s = a_p.x * 2.0;
    float t = s;
    v0 = vec4(t.yxy, s);        // the source is a scalar: every position reads the same value
    t = a_q.x;
    float u = a_q.y;
    u.x = a_q.z;                // all of u: a later read sees nothing of a_q.y
    float w = u;
    u.y = a_q.w;
    v1 = vec4(u, w, t, 1.0);
    float m = a_q.x;
    m.x = a_p.y;                // the same, read past the end of a block
    if (a_p.z > k) v2 = a_q;
    v2 = vec4(m, t, 0.0, 1.0);
    gl_Position = a_p;
}
