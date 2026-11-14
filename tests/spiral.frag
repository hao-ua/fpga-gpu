// Rainbow spiral disk — GLSL ES 1.00 / WebGL 1.0. Uses the iconic Inigo Quilez cosine palette
// (0.5 + 0.5*cos(phase + vec3(0,2,4))) driven by polar angle + radius, with a smoothstep vignette.
precision mediump float;
uniform vec2 uResolution;
void main() {
    vec2 uv = (gl_FragCoord.xy - 0.5 * uResolution.xy) / uResolution.y;  // centered, aspect-correct
    float r = length(uv);
    float a = atan(uv.y, uv.x);                                          // polar angle
    vec3  col = 0.5 + 0.5 * cos(vec3(0.0, 2.0, 4.0) + a * 3.0 + r * 8.0); // spiral rainbow palette
    col *= smoothstep(0.72, 0.70, r);                                    // circular vignette (fade edge)
    gl_FragColor = vec4(col, 1.0);
}
