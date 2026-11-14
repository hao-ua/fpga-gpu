precision mediump float; varying vec4 v0; uniform float n;
void main() { mat3 m = mat3(mat2(1.0, 2.0, 3.0, 4.0)); gl_FragColor = vec4(m[0][1], m[1][0], m[2][2], m[2][0]); }
