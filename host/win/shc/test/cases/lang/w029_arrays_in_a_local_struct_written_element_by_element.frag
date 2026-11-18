precision mediump float; varying vec4 v0; uniform float n;
struct T { float a; vec2 b[2]; };
struct S { float a; float b[3]; T t[2]; };
void main() { S s; s.a = 1.0; s.b[1] = 4.0; s.t[1].b[0] = vec2(5.0, 6.0); gl_FragColor = vec4(s.a, s.b[1], s.b[0], s.t[1].b[0].y); }
