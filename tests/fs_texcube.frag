// fs_texcube.frag — host/fb/fb_texperf.py: one textureCube sample per pixel, the view direction through the screen
// turned about y by the time (an environment-map look-up).
precision mediump float;
uniform samplerCube uCube;
uniform vec4 uScale;                // (field of view scale, turn rate, 1 / aspect, 0)
varying vec4 vUV;
void main() {
    float a = vUV.z * uScale.y;
    vec3 d = vec3(vUV.x * uScale.x, vUV.y * uScale.x, 1.0);
    vec3 dir = vec3(d.x * cos(a) + d.z * sin(a), d.y, d.z * cos(a) - d.x * sin(a));
    gl_FragColor = textureCube(uCube, dir);
}
