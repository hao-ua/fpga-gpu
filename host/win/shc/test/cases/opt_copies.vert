// what block_forward and dead_stores must get right beyond plain copies: writes that are not MOVs, signs that cancel, a register's own value coming back, a loop head
attribute vec4 a_p;
attribute vec4 a_q;
uniform float k;
varying vec4 v0;
varying vec4 v1;
varying vec4 v2;
varying vec4 v3;
varying vec4 v4;
varying vec4 v5;
void main()
{
    vec4 a = a_p;
    vec4 b = a;
    b += a_q;               // a write that is not a MOV: b is no longer a copy of a
    v0 = b - a;
    vec4 c = a_q.wzyx;
    vec4 d = -c;
    vec4 e = -d;            // a negated copy of a negated copy: the signs cancel
    v1 = e * d;
    vec4 f = a_p;
    if (f.x > k) f = a_q;   // ends the block: f is nobody's copy any more
    vec4 g = f.yxwz;
    f = g.wzyx;             // the value comes back under another swizzle: not a copy of itself
    v2 = f + g;
    vec4 h = a_q;
    if (h.x > k) h = a_p;
    vec4 i = h.wzyx;
    h = i.wzyx;             // comes back whole: this one is the identity, and goes
    v3 = h * i;
    v4 = vec4(float(a_p.x > k));
    float x = 1.0;
    int n = 0;
    do {
        x = a_p.y + float(n);   // overwrites what the loop entered with, before any read
        n += 1;
    } while (n < 2);
    v5 = vec4(x);
    gl_Position = a_p;
}
