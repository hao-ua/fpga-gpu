precision mediump float; varying vec4 v0; uniform float n;
struct S { vec4 c; } g;
void main() { g.c = vec4(2.0); gl_FragColor = g.c; }
