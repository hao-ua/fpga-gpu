precision mediump float; varying vec4 v0; uniform float n;
void f(out float x) { x = 5.0; }
void main() { vec2 c = vec2(1.0); f(c[1]); gl_FragColor = vec4(c, 0.0, 0.0); }
//