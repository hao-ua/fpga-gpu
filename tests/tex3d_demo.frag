// tex3d_demo.frag — host/fb/fb_tex3d_demo.py: one 3D texture sample, blended by the texture's alpha. samplerCube as in
// fs_tex3d.frag: the compilers have no sampler3D yet, TEX .xyz is the same and the image bound decides.
precision mediump float;
uniform samplerCube uVol;
varying vec4 vUV;
void main() {
    gl_FragColor = textureCube(uVol, vUV.xyz);
}
