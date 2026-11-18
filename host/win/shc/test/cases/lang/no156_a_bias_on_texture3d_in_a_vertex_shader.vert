#extension GL_OES_texture_3D : enable
uniform sampler3D s; attribute vec3 a;
void main() { gl_Position = texture3D(s, a, 1.0); }
