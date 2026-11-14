// the same in a fragment shader, and copies that must not cross a branch: inside IF only some lanes write, and a block ends there
precision mediump float;
varying vec4 v0;
varying vec4 v1;
uniform float k;
uniform sampler2D t0;
void main()
{
    vec4 a = v0;
    vec4 b = a;
    if (v0.x > 0.25) {
        a = v1;
        b = a * 2.0;
    } else {
        b = b + a;
    }
    vec4 c = b;                 // a copy made after the branch, of a register written in only some lanes
    a = c.wzyx;
    b = a + c;
    vec4 d = texture2D(t0, c.xy);
    vec4 e = d;
    d = d * 0.5;
    if (d.y < k) {
        e.x = -a.y;
        c = e;
    }
    c.w = e.x;
    vec4 f = -c;
    c = f + b;
    gl_FragColor = c + e * f + d;
}
