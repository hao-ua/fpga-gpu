#extension GL_OES_texture_3D : enable
precision mediump float; uniform vec4 u; uniform float f; uniform int ii; varying vec2 w;
uniform sampler3D s;
void main() { gl_FragColor = texture3DLod(s, vec3(w, 0.0), f); }
