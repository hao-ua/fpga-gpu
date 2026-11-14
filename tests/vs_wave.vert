// vs_wave — a displaced vertex: a constant-bounded while loop with continue and break (unrolled by glslc; the 32-bit
// vertex lowering jumps with BRA, the 64-bit one uses the per-lane mask flags of the fragment lowering), SIN, a nested
// if inside the loop, and a varying written in parts.
attribute vec4 aPos;
attribute vec2 aUV;
uniform mat4 uMVP;
uniform float uTime;
uniform float uAmp;
varying vec2 vUV;
varying vec4 vInfo;
void main() {
    float h = 0.0;
    float k = 1.0;
    while (k < 6.0) {
        if (k == 3.0) { k += 1.0; continue; }
        h += uAmp * sin(aPos.x * k + uTime) / k;
        if (h > 2.0) break;
        k += 1.0;
    }
    vec4 p = aPos;
    p.y += h;
    gl_Position = uMVP * p;
    vUV = aUV;
    // one assignment: glslc's component-wise varying writes (vInfo.x = h; vInfo.y = k;) each STV a fresh temporary,
    // so the other components carry stale register contents — a compiler limitation, not what this shader tests
    vInfo = vec4(h, k, aPos.z, aPos.w);
}
