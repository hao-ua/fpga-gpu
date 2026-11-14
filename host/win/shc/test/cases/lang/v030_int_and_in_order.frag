precision mediump float; varying vec4 v0; uniform float n;
void main() { int i = int(n); gl_FragColor = vec4(float(5 * i / 2), float(7 * i / 2 * 2), float((i + 3) / 2 * 2), float(-7 * i / 8)); }
