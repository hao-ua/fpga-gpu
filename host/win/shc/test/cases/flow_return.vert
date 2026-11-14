// main and functions returning in some vertices only: what is kept in locals must still reach its output
attribute vec4 a_p;
attribute vec4 a_q;
varying vec4 v0;
varying vec4 v1;
float f(float x) { if (x > 0.5) return 1.0; return 2.0; }
void main()
{
    v0.xy = a_p.xy;
    v0.zw = a_q.zw;
    v1 = vec4(f(a_q.x));
    gl_Position = a_p;
    if (a_q.y > 0.5) return;
    v0.x = 1.0;
    gl_Position.w = 2.0;
}
