// arrays inside run-time indexed structs: a run-time index under a run-time index, a constant index under a run-time one, a struct in a struct
struct Inner { vec4 c[3]; float w; };
struct Outer { Inner in[2]; mat3 r; vec4 t; };
uniform Outer u_o[4];
uniform float u_s[9];
attribute vec4 a_pos;
attribute vec3 a_i;
varying vec4 v0;
varying float v1;
varying float v2;
void main()
{
    int a = int(a_i.x), b = int(a_i.y), d = int(a_i.z);
    vec4 x = u_o[a].in[b].c[d];
    vec4 y = u_o[a].in[1].c[d];
    vec4 z = u_o[a].in[b].c[2];
    vec4 w = u_o[2].in[b].c[d];
    vec3 q = u_o[d].r * a_pos.xyz;
    gl_Position = vec4(q, 1.0) + u_o[a].t + x * u_o[b].in[0].w;
    v0 = x + y + z + w;
    v1 = u_s[a] + u_s[b];
    v2 = u_s[a + b + d] * u_s[8];
}
