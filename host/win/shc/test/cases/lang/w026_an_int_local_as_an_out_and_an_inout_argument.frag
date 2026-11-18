precision mediump float; varying vec4 v0; uniform float n;
void f(out int a) { a = -1; }
void g(inout int a) { a = -a; }
void main() { int x = 1; f(x); int y = 3; g(y); gl_FragColor = vec4(float(x * int(n)), float(y), 0.0, 0.0); }
