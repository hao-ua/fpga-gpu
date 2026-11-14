precision mediump float;
#define N 2
varying vec3 d[N];
varying vec2 uv;
varying vec4 e[3];
void main() { vec3 c = vec3(0.0); for (int i = 0; i < N; ++i) c += d[i]; gl_FragColor = vec4(c, uv.x) + e[0] + e[2]; }
