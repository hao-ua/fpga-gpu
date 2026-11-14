#extension GL_OES_texture_3D : require
// GL_OES_texture_3D: every overload, a sampler in an array, a struct and a parameter, and the type in a precision statement
precision mediump float;
precision lowp sampler3D;
#ifndef GL_OES_texture_3D
#error the macro is defined
#endif
struct S { sampler3D v; vec3 at; };
uniform sampler3D a;
uniform mediump sampler3D b[2];
uniform S s;
uniform sampler2D c;
varying vec4 v;
vec4 f(sampler3D t, vec3 p) { return texture3D(t, p) * 0.5; }
void main() {
    vec4 r = texture3D(a, v.xyz);
    r += texture3D(a, v.xyz, 1.5);
    r += texture3DProj(a, v);
    r += texture3DProj(a, v, -0.5);
    r += texture3D(b[1], v.zyx);
    r += texture3D(s.v, s.at);
    r += f(a, v.yzx);
    r += texture2D(c, v.xy);
    gl_FragColor = r;
}
