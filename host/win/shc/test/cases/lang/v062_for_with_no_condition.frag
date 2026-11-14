precision mediump float; varying vec4 v0; uniform float n;
void main() { float s = 0.0; for (int i = 0; ; i++) { if (i >= 2) break; s += 1.0; } gl_FragColor = vec4(s); }
