// int / per vertex, both signs, small and large divisors, 24-bit dividends: compiled with noops bit 0 (no integer divider)
// this is the lowered sequence, whose outputs must equal the IDIV program's bit for bit (tools/lower_check.py).
attribute vec4 a0;
attribute vec4 a1;
varying vec4 v0;
varying vec4 v1;
void main() {
    int k = int(a0.x * 500.0) * 4093 - int(a0.y * 977.0);
    int n = (int(a1.x * 37.0) - 50) * 2 + 1;
    int m = (int(a1.y * 20000.0) - 30000) * 2 + 1;
    int q1 = k / n;
    int q2 = k / m;
    int h = q1 / 4096;
    ivec2 iv = ivec2(a0.zw * 300.0) / ivec2(7, -3);
    v0 = vec4(float(h), float(q1 - h * 4096), float(q2), float(k - q1 * n));
    v1 = vec4(vec2(iv), float(n), float(m));
    gl_Position = a0;
}
