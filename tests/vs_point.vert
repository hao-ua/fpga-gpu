// vs_point — a vertex shader for a POINTS draw: gl_PointSize from a per-vertex attribute, a uniform scale and a
// clamp, beside the usual position and a varying. The 64-bit vertex core writes it with STO 1;
// tools/glslc64_check.py checks the row against the 32-bit lowering of the same expression stored as a varying.
attribute vec4 aPos;
attribute vec4 aColor;
attribute float aSize;
uniform mat4 uMVP;
uniform float uPointScale;
varying vec4 vColor;
void main() {
    gl_Position = uMVP * aPos;
    vColor = aColor;
    float s = aSize * uPointScale + 2.0;
    gl_PointSize = clamp(s, 1.0, 64.0);
}
