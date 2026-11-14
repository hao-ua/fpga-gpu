precision mediump float; varying vec4 v0; uniform float n;
struct S { vec4 a; float b; mat2 m; };
void main() { S x = S(vec4(1.0), 2.0, mat2(1.0)); S y = S(vec4(1.0), 2.0, mat2(1.0)); S z = S(vec4(1.0), 3.0, mat2(1.0)); gl_FragColor = vec4(x == y ? 1.0 : 0.0, x == z ? 1.0 : 0.0, x != z ? 1.0 : 0.0, 0.0); }
