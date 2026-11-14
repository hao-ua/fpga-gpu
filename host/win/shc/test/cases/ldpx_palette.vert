// a matrix palette, the shape of a skinned character: mat4 stride 4, a float attribute truncated by int()
uniform mat4 u_bones[24];
uniform mat4 u_proj;
attribute vec4 a_pos;
attribute vec2 a_idx;
attribute vec2 a_weight;
varying vec4 v_c;
void main()
{
    vec4 p = u_bones[int(a_idx.x)] * a_pos * a_weight.x + u_bones[int(a_idx.y)] * a_pos * a_weight.y;
    gl_Position = u_proj * p;
    v_c = p;
}
