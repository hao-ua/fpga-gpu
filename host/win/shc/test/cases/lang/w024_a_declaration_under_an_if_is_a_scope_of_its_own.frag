precision mediump float; varying vec4 v0; uniform float n;
float a = 1.0;
void main() { if (v0.x > 0.0) float a = 5.0; else float a = 7.0; gl_FragColor = vec4(a); }
