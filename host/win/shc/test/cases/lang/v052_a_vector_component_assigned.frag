precision mediump float; varying vec4 v0; uniform float n;
void main() { vec4 c = vec4(0.0); c[2] = 3.0; for (int i = 0; i < 2; i++) c[i] = float(i + 1); gl_FragColor = c; }
