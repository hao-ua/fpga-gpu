// a run-time index into a vector or a matrix is not a uniform array: refused as before
uniform vec4 u_v;
uniform mat4 u_m;
attribute vec4 a_pos;
attribute float a_k;
varying float v0;
void main()
{
    gl_Position = a_pos;
    v0 = u_v[int(a_k)];
}
