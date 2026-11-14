precision mediump float; varying vec4 v0; uniform float n;
void main() { mat3 m = mat3(1.0); m[1] = vec3(5.0, 6.0, 7.0); gl_FragColor = vec4(m[1], m[0][0]); }
