// dowhile_continue.frag — do { .. continue; .. } while (cond): the condition at the loop top, worked out only where a pass has
// run (glslc._do_loop / shc_gen.c do_loop), so a condition that writes runs once a pass; and the bottom-tested form without a continue
precision mediump float;
varying vec4 v0;

float writes(float x)  { float s = 0.0; float k = 0.0; do { s += 1.0; if (x > 0.5) continue; s += 10.0; } while ((k += 1.0) < 3.0); return s + k * 100.0; }
float reads(float x)   { float s = 0.0; float i = 0.0; do { i += 1.0; if (x > 0.5 && i < 3.0) continue; s += 1.0; } while (i < 5.0); return s + i; }
float apart(float x)   { float k = 0.0; do { k += 1.0; if (x > 0.5) continue; } while (k < (x > 0.5 ? 2.0 : 4.0)); return k; }
float counter(float x) { float i = 0.0; float s = 0.0; do { if (x > 0.5) continue; s += 1.0; } while (i++ < 3.0); return s * 10.0 + i; }
float plain(float x)   { float i = 0.0; float s = 0.0; do { s += x; } while (i++ < 3.0); return s + i; }

void main()
{
    float x = v0.x;
    gl_FragColor = vec4(writes(x) + reads(x), apart(x) + counter(x), plain(x), 1.0);
}
