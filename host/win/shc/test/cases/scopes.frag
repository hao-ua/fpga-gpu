// names: a function sees its own, an inner declaration hides an outer one of either kind, and a conditional write belongs to one declaration. host/win/shc/test/flow_check.py has the answers
precision mediump float;
varying vec4 v0;
uniform float n;
const float c = 2.0;
float byval(float x) { return x * 2.0; }
float use_uniform() { return n; }
float use_varying() { return v0.x; }
float use_const() { return c; }
float promotes(float a) { float x = 1.0; x = x + a; return x; }
void set_it(out float x) { x = 2.0; }
float runtime_i(float x) { float s = 0.0; for (float i = 0.0; i < n; i += 1.0) { s += x; } return s; }
float const_i(float x) { float s = 0.0; for (int i = 0; i < 3; i++) { s += x; } return s; }
float cond_i(float x) { float i = 1.0; if (x > 0.5) i = 2.0; return i; }
void main()
{
    float x = 1.0;
    float k = v0.y;
    float y = 1.0;
    float w = 1.0;
    float t = 1.0;
    float c = v0.w;
    float n = 1.0;
    { float x = 2.0; x = 5.0; k += x; }
    { float x = x + 1.0; k += x; }
    { float k = 3.0; y += k; }
    { x = x + 1.0; }
    { float x = 1.0; x = v0.z; k += x; }
    float m = v0.x;
    { float m = m * 2.0; k += m; }
    float q = 1.0;
    { float q = 2.0; k += q; }
    if (v0.x < 0.5) q = 3.0;
    k += q;
    for (int i = 0; i < 2; i++) { float x = 2.0; k += x; }
    for (float i = 0.0; i < v0.x; i += 1.0) { k += 1.0; }
    for (int i = 0; i < 2; i++) { k += 1.0; }
    if (v0.x < 0.5) set_it(y);
    if (v0.y < 0.5) { if (w++ > 0.0) { } }
    float z = 0.0;
    if (v0.z < 0.5) z = (t = 2.0) + 1.0;
    float u = byval(x) + use_uniform() + use_varying() + use_const() + promotes(v0.x) + runtime_i(1.0) + const_i(1.0) + cond_i(v0.x);
    gl_FragColor = vec4(x + k + c, y + n, z + w + t, u);
}
