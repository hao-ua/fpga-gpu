// fs_tex3d.frag — host/fb/fb_tex3d.py: one 3D texture sample per pixel.
#extension GL_OES_texture_3D : enable
precision mediump float;
uniform sampler3D uVol;
uniform vec4 uOff;                  // (s, t, r, 0) at the centre of the screen
uniform vec4 uMul;                  // (s per x, t per y, r per x, r per y), x and y running -1 .. 1 over the screen
varying vec4 vUV;
void main() {
    gl_FragColor = texture3D(uVol, vec3(uOff.x + vUV.x * uMul.x, uOff.y + vUV.y * uMul.y,
                                        uOff.z + vUV.x * uMul.z + vUV.y * uMul.w));
}
