// attribute locations: 18 vectors and a matrix declared, a_7 never read, a matrix read one column of, two pairs read on the two sides of one if
// (aliased when a link binds them to one location, compare.ps1 -Bind); without bindings the active ones take 0, 1, ... in declaration order
uniform mediump float u_A;
attribute mediump vec4 a_0;
attribute mediump vec4 a_1;
attribute mediump vec4 a_2;
attribute mediump vec4 a_3;
attribute mediump vec4 a_4;
attribute mediump vec4 a_5;
attribute mediump vec4 a_6;
attribute mediump vec4 a_7;
attribute mediump mat4 a_m;
attribute mediump vec2 a_8;
attribute mediump vec3 a_9;
attribute mediump float a_10;
varying mediump vec4 v;
void main() {
    vec4 c = a_1 + a_2 * a_3 - a_4 + a_5 * a_6 + a_m[2] + vec4(a_9, a_10);
    if (u_A > 0.0) c += vec4(a_8, a_8); else c += a_0;
    if (u_A > 1.0) c += a_9.xyzx; else c *= a_1;
    v = c;
    gl_Position = a_0;
}
