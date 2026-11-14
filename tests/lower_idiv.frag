// int / per pixel, both signs, small and large divisors: compiled with noops bit 0 (no integer divider) this is the
// lowered sequence, whose outputs must equal the IDIV program's bit for bit (tools/lower_check.py).
precision highp float;
varying vec4 v0;
uniform vec4 uScale;
void main() {
    int k = int(v0.x * 3000.0) * 1021 - int(v0.y * 4500.0);
    int n = (int(v0.z * 40.0) - 60) * 2 + 1;
    int m = int(uScale.x * 9000.0) + 7;
    int q1 = k / n;
    int q2 = k / m;
    ivec2 iv = ivec2(v0.zw * 900.0) / ivec2(-13, 5);
    gl_FragColor = vec4(float(q1 / 2048), float(q2), vec2(iv)) * 0.001;
}
