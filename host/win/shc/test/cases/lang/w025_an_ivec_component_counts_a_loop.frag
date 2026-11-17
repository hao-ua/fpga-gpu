precision mediump float; varying vec4 v0; uniform float n;
void main() { vec4 r = vec4(1.0, 2.0, 3.0, 4.0); for (ivec4 i = ivec4(0, 1, 3, 0); i.x < i.z; i.x += i.y) { r = r.yzwx; } ivec2 k = ivec2(5, 2); k.y -= k.x; gl_FragColor = vec4(r.x, r.y, float(k.y), 0.0); }
