precision mediump float; varying vec4 v0; uniform float n;
void main() { mat2 a = mat2(1.0, 2.0, 3.0, 4.0); mat2 b = mat2(5.0, 6.0, 7.0, 8.0); mat2 m = a; mat2 o = ++m; gl_FragColor = vec4(o[0][0], m[0][0], o[1][1], m[1][1]); }
