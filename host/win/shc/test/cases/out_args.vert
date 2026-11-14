// the same in a vertex shader: gl_Position, gl_PointSize and a swizzled varying as out arguments
attribute vec4 a_p;
attribute vec4 a_q;
varying vec4 v0;
varying vec4 v1;
void place(out vec4 p, vec4 q) { p = q * 2.0; }
void size(out float s) { s = 3.0; }
void fill(out vec2 t, vec2 k) { t = k.yx; }
#define MIX(a, b) ((a) * 0.25 + (b) * 0.75)
void main()
{
    place(gl_Position, a_p);
    size(gl_PointSize);
    v0 = a_q;
    fill(v0.yz, a_p.xy);
    v1 = vec4(MIX(a_p.x,
                  a_q.y));
}
