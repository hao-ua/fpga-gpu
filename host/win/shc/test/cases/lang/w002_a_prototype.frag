precision mediump float; varying vec4 v0; uniform float n;
float g(float);
float h(float a, float);
void main() { gl_FragColor = vec4(g(1.0), h(1.0, 2.0), 0.0, 0.0); }
float g(float a) { return a * 2.0; }
float h(float a, float b) { return a + b; }
