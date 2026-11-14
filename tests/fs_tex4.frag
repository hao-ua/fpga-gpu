// fs_tex4.frag — host/fb/fb_texperf.py: four samples per pixel from four textures (base x detail + lightmap blended by
// a mask: the heaviest common multi-texture layering). uScale.x / uScale.w = the repeats of textures 0, 2 / 1, 3.
precision mediump float;
uniform sampler2D uTex0;
uniform sampler2D uTex1;
uniform sampler2D uTex2;
uniform sampler2D uTex3;
uniform vec4 uScale;                // (repeats tex0 / tex2, scroll, 1 / aspect, repeats tex1 / tex3)
varying vec4 vUV;
void main() {
    vec2 p = vec2(vUV.x * uScale.z, vUV.y);
    vec2 uvA = p * (0.5 * uScale.x) + 0.5 + vec2(vUV.z * uScale.y, 0.0);
    vec2 uvB = p * (0.5 * uScale.w) + 0.5;
    vec4 base   = texture2D(uTex0, uvA);
    vec4 light  = texture2D(uTex1, uvB);
    vec4 detail = texture2D(uTex2, uvA * 3.0);
    vec4 mask   = texture2D(uTex3, uvB + 0.25);
    gl_FragColor = mix(base * light, detail, mask.x * 0.5);
}
