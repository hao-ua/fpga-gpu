precision mediump float;
varying float a; varying vec3 b; varying vec2 c; varying float d; varying vec3 e; varying float f; varying float g; varying vec4 h; varying mat2 m; varying vec2 k[2]; varying float o[3];
void main() { gl_FragColor = vec4(a + b.x * 2.0 + b.y * 3.0 + b.z * 4.0 + c.x * 5.0 + c.y * 6.0 + d * 7.0, e.x + e.y * 2.0 + e.z * 3.0 + f * 4.0 + g * 5.0, h.w + m[1][0] + k[1].y + o[2], m[0][1] + k[0].x + o[1]); }
