attribute vec4 p;
varying vec4 v;
const int N = 3;
void main() { vec4 c = vec4(0.0); int k = 0; float x = 1.0; for (int i = 0; i < N; ++i) { c += p * float(i); } for (int j = 4; j > 0; --j) c.x += 1.0; int a = ++k; float y = --x; c.y += float(a) + y; v = c; gl_Position = p; }
