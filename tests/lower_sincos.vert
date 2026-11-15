// sin / cos over several argument ranges, per vertex: compiled with noops bit 1 (no SIN / COS unit) this is the lowered
// sequence; tools/lower_check.py compares it with the program the unit runs.
attribute vec4 a0;
attribute vec4 a1;
uniform vec4 uScale;
varying vec4 v0;
varying vec4 v1;
varying vec4 v2;
void main() {
    vec4 x = (a0 - 1.5) * vec4(1.0, 3.0, 40.0, 1000.0);
    vec4 t = (a1 - 1.5) * 0.001;
    v0 = sin(x);
    v1 = cos(x);
    v2 = vec4(sin(t.x), cos(t.y), sin(x.z) * cos(x.w), sin(t.z) + cos(t.w));
    gl_Position = vec4(sin(x.x * uScale.x), cos(x.y * uScale.y), 0.0, 1.0);
}
