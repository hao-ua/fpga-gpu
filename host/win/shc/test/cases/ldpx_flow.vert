// run-time indices under control flow and in a loop, with a loop variable the compiler cannot unroll, and in a function
uniform vec4 u_c[32];
uniform mat4 u_m[4];
uniform int u_n;
attribute vec4 a_pos;
attribute float a_k;
varying vec4 v0;
vec4 pick(int i, float s)
{
    if (s > 0.5) return u_c[i + 1];
    return u_c[i] * s;
}
void main()
{
    vec4 sum = vec4(0.0);
    for (int i = 0; i < 4; i++) sum += u_c[i * 3 + int(a_k)];
    if (a_k > 2.0) sum += pick(int(a_k), a_pos.x);
    else sum -= pick(u_n, a_pos.y);
    gl_Position = u_m[a_k > 1.0 ? 0 : 1] * a_pos;
    v0 = sum;
}
