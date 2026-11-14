precision mediump float; varying vec4 v0; uniform float n;
void main() { gl_FragColor = vec4(mod(vec2(5.0, 7.0), vec2(3.0, 4.0)), mod(vec2(5.0, 7.0), 3.0)); }
