precision mediump float;
varying vec4 v;
uniform float t;
uniform int n;
uniform bool flag;
uniform bvec2 bb;
uniform ivec3 iv;
void main() {
    vec4 c = vec4(0.0);
    for (int i = 0; i < 8; i++) {
        float k = float(i) / 8.0;
        if (k > t) break;
        if (mod(float(i), 2.0) < 1.0) continue;
        for (int j = 3; j >= 0; j--) c += v * k * float(j);
    }
    if (flag) { c.r = 1.0; } else if (bb.x || bb.y) { c.g = 1.0; } else { c.b = float(iv.y + n); }
    if (!(c.a < 0.5) && c.r >= 0.25) c.a = 1.0;
    gl_FragColor = c;
}