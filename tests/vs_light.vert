// vs_light — a vertex shader with the constructs the two vertex lowerings differ on: an if / else on a per-vertex value
// (BRP on the 32-bit core, IF / ELSE on the 16-lane core) and a runtime-bounded loop with a break (BRP / BRA against
// LOOP / BREAK). tools/glslc64_check.py compiles it both ways and compares 16 random vertices.
attribute vec4 aPos;
attribute vec3 aNormal;
attribute vec2 aUV;
attribute vec4 aColor;
uniform mat4 uMVP;
uniform mat3 uNormalMat;
uniform vec3 uLightDir;
uniform float uAmbient;
uniform int uSteps;
varying vec4 vColor;
varying vec2 vUV;
void main() {
    gl_Position = uMVP * aPos;
    vec3 n = normalize(uNormalMat * aNormal);
    float d = max(dot(n, normalize(uLightDir)), 0.0);
    vec4 c = aColor * (uAmbient + d);
    if (aPos.x > 0.5) {
        c.r = 1.0 - c.r;
    } else {
        c.g = c.g * 0.5;
    }
    for (int i = 0; i < uSteps; i++) {
        c.b += 0.125;
        if (c.b > 1.0) break;
    }
    vColor = c;
    vUV = aUV * 2.0 - 1.0;
}
