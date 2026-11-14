precision mediump float; varying vec4 v0; uniform float n;
void main() { int k = int(v0.x * 4.0); float a[4]; for (int i = 0; i < 4; i++) a[i] = 0.0; a[k] = 1.0; gl_FragColor = vec4(a[0], a[1], a[2], a[3]); }
