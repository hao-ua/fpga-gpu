precision mediump float; varying vec4 v0; uniform float n;
const vec2 c = vec2(1.0);
vec2 f() { return c; }
void main() { vec2 c = vec2(2.0); gl_FragColor = vec4(c, f()); }
