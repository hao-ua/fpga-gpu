attribute vec4 a0;
uniform float s[200]; uniform vec2 r[60]; varying vec2 t;
void main() { int i = int(a0.x * 0.0) + 199; int j = int(a0.y * 0.0) + 59; gl_Position = a0; t = vec2(s[i] + s[0], r[j].y); }
