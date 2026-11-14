precision mediump float; varying vec4 v0; uniform float n;
void f(out float a) { a = 1.0; }
void f(out vec2 a) { a = vec2(2.0); }
void main() { float x = 0.0; vec2 y = vec2(0.0); if (v0.x > 0.5) f(x); else f(y); gl_FragColor = vec4(x, y, 0.0); }
