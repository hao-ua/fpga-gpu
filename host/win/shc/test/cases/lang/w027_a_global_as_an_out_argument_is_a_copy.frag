precision mediump float; varying vec4 v0; uniform float n;
float x = 1.0;
bool d(out float y) { y = 2.0; return x == 1.0 && y == 2.0; }
void main() { bool r = d(x); gl_FragColor = vec4(r ? 1.0 : 0.0, x, 0.0, 0.0); }
