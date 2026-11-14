precision mediump float; varying vec4 v0; uniform float n;
void main() { int k = int(v0.x * 4.0); vec4 c = vec4(0.0); c[k] = 1.0; gl_FragColor = c; }
