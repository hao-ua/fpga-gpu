precision mediump float; varying vec4 v0; uniform float n;
void main() { ivec2 w = ivec2(7, -7) / ivec2(2, 2); ivec2 u = ivec2(9, 8) / 4; gl_FragColor = vec4(vec2(w), vec2(u)); }
