precision mediump float; varying vec4 v0; uniform float n;
float f(float a) { return a + 1.0; }
vec2 f(vec2 a) { return a + 2.0; }
int f(int a) { return a + 3; }
void main() { gl_FragColor = vec4(f(1.0), f(vec2(1.0)), float(f(1))); }
