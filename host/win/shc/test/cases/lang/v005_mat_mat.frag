precision mediump float; varying vec4 v0; uniform float n;
void main() { mat2 p = mat2(1.0, 2.0, 4.0, 8.0); mat2 c = mat2(2.0, 2.0, 2.0, 2.0) / p; gl_FragColor = vec4(c[0], c[1]); }
