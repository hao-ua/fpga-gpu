precision mediump float; varying vec4 v0; uniform float n;
void neg(inout float a[2]) { a[0] = -a[0]; a[1] = -a[1]; }
float f(inout float a[2], inout float b[2]) { neg(a); return a[0] == b[0] ? 1.0 : -1.0; }
void main() { float r[2]; r[0] = 3.0; r[1] = 4.0; float m = f(r, r); gl_FragColor = vec4(m, 0.0, 0.0, 0.0); }
