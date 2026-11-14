precision mediump float; varying vec4 v0; uniform float n;
void main() { float i = v0.x; float o = ++i; gl_FragColor = vec4(o, i, 0.0, 0.0); }
