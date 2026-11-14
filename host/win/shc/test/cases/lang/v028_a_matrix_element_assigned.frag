precision mediump float; varying vec4 v0; uniform float n;
void main() { mat3 m = mat3(1.0); m[2][1] = 3.0; m[0].yz = vec2(8.0, 9.0); gl_FragColor = vec4(m[2][1], m[0]); }
