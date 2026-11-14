// fs_tex1.frag — host/fb/fb_texperf.py: one texture2D sample per pixel. uv spans uScale.x texture repeats over the
// screen height (uScale.z = 1 / aspect squares the mapping), scrolled by the time (vUV.z) at uScale.y per second.
precision mediump float;
uniform sampler2D uTex0;
uniform vec4 uScale;                // (repeats over the screen, scroll, 1 / aspect, 0)
varying vec4 vUV;                   // (x * aspect, y, time, 1)
void main() {
    vec2 uv = vec2(vUV.x * uScale.z, vUV.y) * (0.5 * uScale.x) + 0.5 + vec2(vUV.z * uScale.y, 0.0);
    gl_FragColor = texture2D(uTex0, uv);
}
