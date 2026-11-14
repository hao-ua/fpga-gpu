precision mediump float; varying vec4 v0; uniform float n;
void main() { int m = int(n); int i = 0; float s = 0.0; while (i < m) { s += 1.0; i++; } gl_FragColor = vec4(s); }
