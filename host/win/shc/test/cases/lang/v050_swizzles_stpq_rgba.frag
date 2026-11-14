precision mediump float; varying vec4 v0; uniform float n;
void main() { vec4 w = vec4(1.0, 2.0, 3.0, 4.0); w.st += vec2(10.0, 20.0); w.pq -= vec2(1.0, 1.0); gl_FragColor = w.abgr; }
