precision mediump float; varying vec4 v0; uniform float n;
void main() { int t = int(n) + 2; ivec2 w = ivec2(t * 7, t * 3) / ivec2(7, 3); gl_FragColor = vec4(vec2(w), 0.0, 0.0); }
