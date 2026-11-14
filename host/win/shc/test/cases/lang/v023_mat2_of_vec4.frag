precision mediump float; varying vec4 v0; uniform float n;
void main() { mat2 m = mat2(vec4(1.0, 2.0, 3.0, 4.0)); gl_FragColor = vec4(m[0], m[1]); }
