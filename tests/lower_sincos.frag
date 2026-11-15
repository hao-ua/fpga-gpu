// sin / cos over several argument ranges, per pixel: compiled with noops bit 1 (no SIN / COS unit) this is the lowered
// sequence; tools/lower_check.py compares it with the program the unit runs.
precision highp float;
varying vec4 v0;
uniform vec4 uScale;
void main() {
    vec4 x = (v0 - 1.0) * vec4(2.0, 7.0, 90.0, 3000.0);
    gl_FragColor = sin(x) * 0.5 + cos(x * uScale) * 0.25;
}
