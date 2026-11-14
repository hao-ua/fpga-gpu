precision mediump float; varying vec4 v0; uniform float n;
void main() { int k = int(v0.x * 4.0); vec4 w = vec4(10.0, 20.0, 30.0, 40.0); gl_FragColor = vec4(w[k]); }
