#extension GL_OES_texture_3D : enable
precision mediump float; uniform vec4 u; uniform float f; uniform int ii; varying vec2 w;
uniform sampler3D s;
void main() { vec4 c = texture3DProjLod(s, vec4(w, 0.0, 1.0), 0.0); }
