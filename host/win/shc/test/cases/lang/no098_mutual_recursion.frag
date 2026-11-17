precision mediump float; uniform vec4 u; uniform float f; uniform int ii; varying vec2 w;
float h(float a);
float g(float a) { return h(a); }
float h(float a) { return g(a); }
void main() { }
