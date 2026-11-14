precision mediump float; varying vec4 v0; uniform float n;
void main() { mat2 a = mat2(1.0, 2.0, 3.0, 4.0); mat2 b = mat2(5.0, 6.0, 7.0, 8.0); gl_FragColor = vec4(vec2(1.0, 1.0) * a, 0.0, 0.0); }
