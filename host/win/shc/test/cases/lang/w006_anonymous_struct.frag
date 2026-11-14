precision mediump float; varying vec4 v0; uniform float n;
struct { vec2 p; } g;
void main() { g.p = vec2(3.0); struct { float q; } l; l.q = 1.0; gl_FragColor = vec4(g.p, l.q, 0.0); }
