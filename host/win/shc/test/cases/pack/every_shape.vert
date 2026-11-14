attribute vec4 a0;
varying float a; varying vec3 b; varying vec2 c; varying float d; varying vec3 e; varying float f; varying float g; varying vec4 h; varying mat2 m; varying vec2 k[2]; varying float o[3];
void main() { gl_Position = a0; a = 1.0; b.xy = vec2(2.0, 3.0); b.z = b.y + 1.0; c = vec2(5.0, 6.0); d = 7.0; e = vec3(8.0, 9.0, 10.0); f = 11.0; g = 12.0; h = vec4(13.0, 14.0, 15.0, 16.0);
  m = mat2(17.0, 18.0, 19.0, 20.0); k[0] = vec2(21.0, 22.0); k[1] = vec2(23.0, 24.0); o[0] = 25.0; o[1] = 26.0; o[2] = 27.0; }
