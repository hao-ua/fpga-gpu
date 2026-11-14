precision mediump float; varying vec4 v0; uniform float n;
void main() { int p = 7; int q = p / 2; int r = q; r /= 2; gl_FragColor = vec4(float(q), float(r), 0.0, 0.0); }
