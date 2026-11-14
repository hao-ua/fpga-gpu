// copies whose source is written afterwards, in whole and in part: what block_forward must notice
attribute vec4 a_p;
attribute vec4 a_q;
varying vec4 v0;
varying vec4 v1;
varying vec4 v2;
varying vec4 v3;
void main()
{
    vec4 a = a_p;
    vec4 b = a;             // a copy
    a = a_q * 2.0;          // its source written: b keeps the old value
    vec4 c = b.wzyx;        // a swizzled copy
    b = a + b;
    vec4 d = -c;            // a negated copy
    c.x = 1.0;              // a masked write of the source: only d's matching component would change
    v0 = d + c;
    vec4 e = d;
    d.yz = a.xy;            // two components of e's source
    v1 = e * d;
    a = a.yzwx;             // a register written from itself, swizzled
    float s = a.x;          // a scalar copy
    a.x = 2.0;
    float t = s;
    s = a.y;
    v2 = vec4(s, t, b.w, c.y);
    vec4 f = a;
    vec4 g = f;
    f = g.yxwz;             // a swap through a copy
    g = f.yxwz;
    v3 = f - g;
    gl_Position = a_p;
}
