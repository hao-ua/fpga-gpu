// fs_spiral.frag — host/fb/fb_shader.py: tests/spiral.frag's rainbow spiral (cosine palette over polar angle and
// radius, smoothstep vignette) turning with the time, the screen position from the vertex shader's varying.
// No loop: length, atan (the compiler's minimax polynomial + reductions), 3 x COS, smoothstep.
precision mediump float;
varying vec4 vUV;                   // (x * aspect, y, time, 1)
void main() {
    vec2 uv = vUV.xy * 0.5;
    float r = length(uv);
    float a = atan(uv.y, uv.x) + vUV.z;                                   // polar angle, turning
    vec3  col = 0.5 + 0.5 * cos(vec3(0.0, 2.0, 4.0) + a * 3.0 + r * 8.0); // spiral rainbow palette
    col *= smoothstep(0.52, 0.50, r);                                    // circular vignette (fade edge)
    gl_FragColor = vec4(col, 1.0);
}
