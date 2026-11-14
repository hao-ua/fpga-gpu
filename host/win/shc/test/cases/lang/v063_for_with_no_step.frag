precision mediump float; varying vec4 v0; uniform float n;
void main() { float s = 0.0; for (int i = 0; i < 3; ) { s += 1.0; i++; } gl_FragColor = vec4(s); }
