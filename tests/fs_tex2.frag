// fs_tex2.frag — host/fb/fb_texperf.py: two samples per pixel (a detail texture modulated by a lightmap: the
// classic two-layer surface). uScale.x / uScale.w = the repeats of texture 0 / 1 over the screen height.
precision mediump float;
uniform sampler2D uTex0;
uniform sampler2D uTex1;
uniform vec4 uScale;                // (repeats tex0, scroll, 1 / aspect, repeats tex1)
varying vec4 vUV;
void main() {
    vec2 p = vec2(vUV.x * uScale.z, vUV.y);
    vec2 uv0 = p * (0.5 * uScale.x) + 0.5 + vec2(vUV.z * uScale.y, 0.0);
    vec2 uv1 = p * (0.5 * uScale.w) + 0.5;
    gl_FragColor = texture2D(uTex0, uv0) * texture2D(uTex1, uv1);
}
