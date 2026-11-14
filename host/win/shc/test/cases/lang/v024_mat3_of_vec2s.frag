precision mediump float; varying vec4 v0; uniform float n;
void main() { vec2 w = vec2(7.0, 8.0); mat3 m = mat3(w, 1.0, w, 2.0, w, 3.0); gl_FragColor = vec4(m[0][2], m[1][2], m[2][2], m[2][1]); }
