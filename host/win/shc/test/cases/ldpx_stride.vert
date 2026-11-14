// every stride the code generator treats differently: 1, 2 and 4 by IADD, 3 by two, 5 and up by IMUL, and ints stored as floats
struct L2 { vec4 a; vec4 b; };
struct L3 { vec4 a; vec2 b; float c; };
struct L5 { mat4 m; vec4 c; };
uniform vec4 u_v[16];
uniform float u_f[16];
uniform int u_i[8];
uniform mat2 u_m2[6];
uniform mat3 u_m3[6];
uniform L2 u_l2[6];
uniform L3 u_l3[6];
uniform L5 u_l5[6];
uniform int u_k;
attribute vec4 a_pos;
attribute float a_n;
varying vec4 v0;
varying vec4 v1;
void main()
{
    int n = int(a_n);
    int m = u_i[n] + 1;
    vec2 q = u_m2[n] * a_pos.xy;
    vec3 r = u_m3[m] * a_pos.xyz;
    vec4 s = u_l2[n].a + u_l2[m].b;
    vec4 t = u_l3[n].a * u_l3[m].c + vec4(u_l3[n].b, 0.0, 1.0);
    gl_Position = u_l5[n].m * a_pos + u_l5[m].c;
    v0 = u_v[n + 1] + u_v[n * 2 + u_k] + vec4(u_f[n], u_f[m], u_f[n + m], q.x);
    v1 = s + t + vec4(r, q.y);
}
