precision mediump float; varying vec4 v0; uniform float n;
void main() { gl_FragColor = vec4(smoothstep(vec2(0.0, 0.0), vec2(2.0, 4.0), vec2(1.0, 2.0)), smoothstep(vec2(0.0, 4.0), vec2(1.0, 8.0), vec2(0.0, 8.0))); }
