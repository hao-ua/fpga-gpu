// counted_loops.frag — long constant loops whose body never reads the index are `LOOP n` with no counter (glslc._counted_loop /
// shc_gen.c counted_loop), `if (c) break;` and `if (c) continue;` are SETP + BREAK / CONT and `a || b` a pair each (_ctl_if /
// ctl_if). With the shapes that keep the older forms: a break beside other statements or with an else, the index's name read in the
// body (a nested loop of that name, a write to it), and a parameter hiding the global constant the pre-pass counted with.
precision mediump float;
varying vec4 v0;
const int N = 33;

float orbrk(float x)   { float s = 0.0; for (int i = 0; i < 40; i++) { s += x; if (s > 30.5 || (x > 0.5 && s > 2.5)) break; } return s; }
float orside(float x)  { float k = 0.0; for (int i = 0; i < 20; i++) { if (x > 0.5 || (k += 1.0) > 100.0) break; } return k; }
float or3(float x)     { float s = 0.0; for (int i = 0; i < 30; i++) { s += 1.0; if (s > 25.5 || (x > 0.5 && s > 1.5) || s < 0.0) break; } return s; }
float orcont(float x)  { float s = 0.0; float k = 0.0; for (int i = 0; i < 20; i++) { k += 1.0; if (x > 0.5 || k > 15.5) continue; s += 1.0; } return s + k; }
float blk(float x)     { float s = 0.0; for (int i = 0; i < 40; i++) { s += 1.0; if (x > 0.5 && s > 2.5) { { break; } } } return s; }
float beside(float x)  { float s = 0.0; for (int i = 0; i < 30; i++) { if (x > 0.5 && s > 1.5) { s += 10.0; break; } s += 1.0; } return s; }
float withelse(float x) { float s = 0.0; for (int i = 0; i < 40; i++) { if (x > 0.5 && s > 2.5) break; else s += 1.0; } return s; }
float inner(float x)   { float s = 0.0; for (int i = 0; i < 40; i++) { if (x > 0.5) { if (s > 2.5) break; s += 1.0; } else s += 2.0; } return s; }
float again(float x)   { float s = 0.0; for (int i = 0; i < 20; i++) { for (int i = 0; i < 2; i++) { s += 1.0; } if (x > 0.5 && s > 3.5) break; } return s; }
float writes(float x)  { float s = 0.0; for (int i = 0; i < 40; i++) { s += 1.0; if (x > 0.5 && s > 2.5) { i = 100; } } return s; }
float hides(float x, int N) { float s = 0.0; for (int i = 0; i < N; i++) { s += 1.0; if (x > 0.5 && s > 1.5) break; } return s; }
float ret100(float x)  { float t = 0.0; for (int i = 0; i < 100; i++) { t += 1.0; if (x > 0.5 && t > 6.5) return t; } return t + 0.5; }
float plain70(float x) { float s = 0.0; for (int i = 0; i < 70; i++) { s += x; } return s; }

void main()
{
    float x = v0.x;
    float a = orbrk(x) + orside(x) + or3(x) + orcont(x) + blk(x);
    float b = beside(x) + withelse(x) + inner(x) + again(x) + writes(x);
    float c = hides(x, 20) + ret100(x) + plain70(x);
    gl_FragColor = vec4(a, b, c, 1.0);
}
