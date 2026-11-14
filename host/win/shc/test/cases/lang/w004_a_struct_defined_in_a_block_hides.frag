precision mediump float; varying vec4 v0; uniform float n;
struct S { float a; };
void main() { S o = S(1.0); float r = 0.0; { struct S { float b; float c; }; S t = S(2.0, 3.0); r = t.c; } gl_FragColor = vec4(o.a, r, 0.0, 0.0); }
