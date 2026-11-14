precision mediump float; varying vec4 v0; uniform float n;
void main() { int i = int(n); gl_FragColor = vec4(i == 4 ? 1.0 : 0.0, i != 4 ? 1.0 : 0.0, 4 == i ? 1.0 : 0.0, i + 1 == 4 ? 1.0 : 0.0); }
