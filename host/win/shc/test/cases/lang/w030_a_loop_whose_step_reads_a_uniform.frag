precision mediump float; varying vec4 v0; uniform float n;
float f() { for (int i = 1; i < 10; i += int(n) - 4) return 2.0; return 3.0; }
void main() { gl_FragColor = vec4(f()); }
