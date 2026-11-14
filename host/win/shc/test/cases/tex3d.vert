#extension GL_OES_texture_3D : enable
// no vertex texture units: a sampler compiles, the link refuses it
uniform sampler3D a;
attribute vec4 p;
void main() {
    gl_Position = texture3DLod(a, p.xyz, 1.0) + texture3DProjLod(a, p, 0.0);
}
