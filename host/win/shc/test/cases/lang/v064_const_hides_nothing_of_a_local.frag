precision mediump float; varying vec4 v0; uniform float n;
const float K = 3.0;
void main() { vec2 c = vec2(2.0); gl_FragColor = vec4(c, K, K); }
