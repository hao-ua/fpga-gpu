precision mediump float; varying vec4 v0; uniform float n;
void main() { float a = 1.0; float r = v0.x > 0.5 ? (a = 2.0) : 3.0; gl_FragColor = vec4(r, a, 0.0, 0.0); }
