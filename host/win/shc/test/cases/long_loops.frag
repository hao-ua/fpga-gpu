// long_loops.frag — constant-bounded for loops either side of the rule that makes a long one a hardware loop
// (glslc._long_for / shc_gen.c long_for: more than 16 passes with a break or continue, more than 64 without, unless
// the body needs the index as a constant). Every form the pass count is read from, and the ones that are not that form.
precision mediump float;
varying vec4 v0;
uniform float w[20];
const int N = 33;
const float LIM = 9.0;

float pick(int k) { return w[k]; }

float brk40(float x)    { float s = 0.0; for (int i = 0; i < 40; i++) { s += x; if (s > 2.5) break; } return s; }
float cont20(float x)   { float s = 0.0; for (int i = 0; i < 20; i++) { if (x > 0.5) continue; s += 1.0; } return s; }
float brk16(float x)    { float s = 0.0; for (int i = 0; i < 16; i++) { s += x; if (s > 2.5) break; } return s; }
float brk17(float x)    { float s = 0.0; for (int i = 0; i <= 16; i++) { s += x; if (s > 2.5) break; } return s; }
float plain64(float x)  { float s = 0.0; for (int i = 0; i < 64; i++) { s += x; } return s; }
float plain65(float x)  { float s = 0.0; for (int i = 0; i < 65; ++i) { s += x; } return s; }
float down(float x)     { float s = 0.0; for (int i = N; i > 0; i--) { s += x; if (s > LIM) break; } return s; }
float downto(float x)   { float s = 0.0; for (int i = 40; i >= -3; i -= 2) { s += x; if (s > LIM) break; } return s; }
float fstep(float x)    { float s = 0.0; for (float t = 0.0; t < 10.0; t += 0.5) { s += x; if (t > x * 8.0) break; } return s; }
float noteq(float x)    { float s = 0.0; for (int i = 0; i != 24; i += 1) { s += x; if (s > 2.5) break; } return s; }
float asvalue(float x)  { float s = 0.0; for (int i = 0; i < 30; i++) { s += float(i) * x; if (i == 7) break; } return s; }
float indexed(float x)  { float s = 0.0; for (int i = 0; i < 20; i++) { s += w[i] * x; if (s > 2.5) break; } return s; }
float handed(float x)   { float s = 0.0; for (int i = 0; i < 20; i++) { s += pick(i) * x; if (s > 2.5) break; } return s; }
float early(float x)    { float s = 0.0; for (int i = 0; i < 48; i++) { s += x; if (s > 4.5) return s; if (s > 9.5) break; } return s + 100.0; }
float nested(float x)   { float s = 0.0; for (int i = 0; i < 20; i++) { for (int j = 0; j < 20; j++) { s += x; if (j == 1) break; } if (s > 9.5) break; } return s; }
float hidden(float x)   { int N = 5; float s = 0.0; for (int i = 0; i < N; i++) { s += x; if (s > 2.5) break; } return s; }       // a local N hides the global one
float param(float x, int N) { float s = 0.0; for (int i = 0; i < N; i++) { s += x; if (s > 2.5) break; } return s; }                // a parameter does too, unseen by the pre-pass
float twovars(float x)  { float s = 0.0; for (int i = 0, j = 0; i < 20; i++) { s += x; if (s > 2.5) break; } return s; }
float bodywrites(float x) { float s = 0.0; for (int i = 0; i < 40; i++) { s += x; if (s > 2.5) { i = 100; } } return s; }

void main()
{
    float x = v0.x;
    float a = brk40(x) + cont20(x) + brk16(x) + brk17(x) + plain64(x) + plain65(x);
    float b = down(x) + downto(x) + fstep(x) + noteq(x) + asvalue(x);
    float c = indexed(x) + handed(x) + early(x) + nested(x) + hidden(x) + param(x, 3) + twovars(x) + bodywrites(x);
    gl_FragColor = vec4(a, b, c, 1.0);
}
