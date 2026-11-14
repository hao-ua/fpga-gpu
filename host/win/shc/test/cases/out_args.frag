// out and inout arguments with no register of their own, and macro calls over several lines
precision mediump float;
varying vec4 v0;
#define PICK(a, b) vec4(a, b, 0.0, 1.0)
#define ADD3(x, y, z) ((x) + (y) + (z))
#define ONE(x) x
void paint(out vec4 c, in vec4 v)
{
    if (v.x > 0.5) {
        c = v.wzyx;
        return;
    }
    c = v;
}
void halve(inout vec2 p) { p = p.yx * 0.5; }
void one(out float x, float k) { x = k + 1.0; }
void both(inout vec4 q, out float w) { w = q.x; q.x = 2.0; }
void main()
{
    vec4 a = PICK(v0.x,
                  v0.y);
    vec4 b = PICK
        (v0.z, v0.w);
    float s = ADD3(v0.x,
                   ONE(v0.y
                       ),

                   v0.z);
    halve(a.xy);
    halve(b.zx);
    one(a.w, s);
    one(b.y, a.w);
    vec4 c = a;
    both(c, c.w);
    paint(gl_FragColor, a + b * s + c);
}
