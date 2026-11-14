precision mediump float; varying vec4 v0; uniform float n;
const float s = sin(0.0) + cos(0.0);
const int m = int(s) + 1;
float a[m > 1 ? 2 : 1];
void main() { a[1] = s; gl_FragColor = vec4(a[1], float(m), 0.0, 0.0); }
