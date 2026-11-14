#define N 2
attribute vec4 p;
varying vec3 d[N];
varying vec2 uv;
varying vec4 e[3];
void main() { for (int i = 0; i < N; ++i) { d[i] = p.xyz * float(i); } e[0] = p; e[2].xy = p.zw; e[2].zw = p.xy; uv = p.xy; gl_Position = p; }
