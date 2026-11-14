precision mediump float; varying vec4 v0; uniform float n;
void main() { float a = 1.0; bool b = v0.x > 0.5 || (a = 2.0) > 0.0; gl_FragColor = vec4(a, b ? 1.0 : 0.0, 0.0, 0.0); }
