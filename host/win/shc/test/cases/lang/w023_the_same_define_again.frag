precision mediump float; varying vec4 v0; uniform float n;
#define V (1.0 + 2.0)
#define V (1.0 /* */ + 2.0)
#define G(a,b) (a  +b)
#define G(a,b)(a /* */ +b)
void main() { gl_FragColor = vec4(V, G(1.0, 1.0), 0.0, 0.0); }
