precision mediump float; varying vec4 v0; uniform float n;
void main() { mat4 m = mat4(mat2(1.0, 2.0, 3.0, 4.0)); gl_FragColor = vec4(m[1][1], m[2][2], m[3][3], m[3][0]); }
