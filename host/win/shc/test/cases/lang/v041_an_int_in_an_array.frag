precision mediump float; varying vec4 v0; uniform float n;
void main() { int a[2]; a[1] = 5; a[0] = a[1] + 1; gl_FragColor = vec4(float(a[0]), float(a[1]), 0.0, 0.0); }
