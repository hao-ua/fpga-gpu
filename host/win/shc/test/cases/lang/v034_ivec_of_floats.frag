precision mediump float; varying vec4 v0; uniform float n;
void main() { ivec2 w = ivec2(vec2(1.75, -1.75)); ivec3 u = ivec3(2.5); gl_FragColor = vec4(float(w.x), float(w.y), float(u.z), 0.0); }
