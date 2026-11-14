precision mediump float; varying vec4 v0; uniform float n;
void main() { struct I { int i; float f; }; I s; s.i = 3; s.f = 1.0; I t = I(5, 2.0); I u = t; gl_FragColor = vec4(float(s.i + 1), float(t.i + 1), float(u.i * 2), 0.0); }
