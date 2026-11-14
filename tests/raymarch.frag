// Raymarched sphere on a floor with diffuse lighting — a genuinely heavy procedural shader:
// signed-distance field, a sphere-tracing loop with early-out break, central-difference normals
// (6 extra SDF evals), Lambert lighting, and gamma correction. GLSL ES 1.00.
precision mediump float;
uniform vec2  uResolution;
uniform float uSteps;               // marching steps (runtime -> hardware loop, compact code)

float sdSphere(vec3 p, float r) { return length(p) - r; }

float map(vec3 p) {                 // scene SDF = union(sphere, ground plane)
    float s     = sdSphere(p - vec3(0.0, 0.0, 0.0), 1.0);
    float plane = p.y + 1.0;
    return min(s, plane);
}

vec3 calcNormal(vec3 p) {           // gradient of the SDF via central differences
    vec2 e = vec2(0.008, 0.0);
    return normalize(vec3(
        map(p + e.xyy) - map(p - e.xyy),
        map(p + e.yxy) - map(p - e.yxy),
        map(p + e.yyx) - map(p - e.yyx)));
}

void main() {
    vec2 uv = (gl_FragCoord.xy - 0.5 * uResolution.xy) / uResolution.y;
    vec3 ro = vec3(0.0, 0.6, 3.0);                   // camera
    vec3 rd = normalize(vec3(uv, -1.4));             // ray direction

    float t = 0.0;
    float hit = 0.0;
    for (int i = 0; i < int(uSteps); i++) {          // sphere tracing
        vec3  p = ro + rd * t;
        float d = map(p);
        if (d < 0.002) { hit = 1.0; break; }         // surface hit
        t += d;
        if (t > 12.0) break;                         // escaped to background
    }

    vec3 col = vec3(0.55, 0.72, 0.95);               // sky gradient base
    col -= 0.3 * uv.y;
    if (hit > 0.5) {
        vec3 p   = ro + rd * t;
        vec3 n   = calcNormal(p);
        vec3 lig = normalize(vec3(0.7, 0.8, 0.5));
        float dif = max(dot(n, lig), 0.0);
        float amb = 0.35;
        // sphere is warm, floor is checker-tinted by world position
        vec3 base = (p.y < -0.9) ? vec3(0.35, 0.55, 0.35) : vec3(0.95, 0.45, 0.30);
        col = base * (dif + amb);
    }
    col = pow(col, vec3(0.4545));                    // gamma 2.2
    gl_FragColor = vec4(col, 1.0);
}
