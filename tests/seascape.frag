// Seascape for gpu2 — a COMPACT reinterpretation of Alexander Alekseev (TDM)'s classic Shadertoy
// "Seascape", rebuilt to fit this GPU's fragment core (31 registers, 4096-instruction IRAM, FP24).
//
// TDM's original also compiles as written (hardware loops for constant counts, the constant pool pruned: 3 766
// instructions, 48 constants) and runs through host/win/tests/gles_shader, which takes the Shadertoy source as it is.
// This compact version keeps the *character* — a raymarched wave heightfield, sky+sun, fresnel sea colour and sun
// specular/glitter, gamma — with a cheap sum-of-sines ocean and analytic normals, as a test of its own.
//
// gpu2 conventions: real uniforms (no iResolution/iTime), gl_FragCoord/gl_FragColor, and a *uniform* step
// count (uSteps) so the march stays a hardware LOOP rather than unrolling past IRAM.
precision mediump float;
uniform vec2  uResolution;
uniform float uTime;
uniform float uSteps;                 // raymarch step budget (runtime -> hardware loop)

const float SEA_LEVEL = 0.0;

// Ocean height at world xz: three travelling sine waves (cheap, FP16-safe, few constants).
float waveHeight(vec2 p) {
    float t = uTime;
    float h  = sin(p.x*0.60 + t*1.30) * 0.50;
    h += sin(p.y*0.85 - t*1.10) * 0.32;
    h += sin((p.x + p.y)*1.30 + t*2.10) * 0.18;
    return h * 0.55;
}

// Scene distance (signed height above the ocean surface).
float map(vec3 p) { return p.y - SEA_LEVEL - waveHeight(p.xz); }

// Surface normal via forward differences of the height field (3 taps -> low register pressure).
vec3 getNormal(vec3 p) {
    float e = 0.06;
    float h0 = waveHeight(p.xz);
    float hx = waveHeight(vec2(p.x + e, p.z));
    float hz = waveHeight(vec2(p.x, p.z + e));
    return normalize(vec3(h0 - hx, e, h0 - hz));
}

// Sky: horizon-to-zenith gradient plus a soft sun disk/glow.
vec3 getSkyColor(vec3 rd, vec3 sun) {
    float up = max(rd.y, 0.0);
    vec3 sky = mix(vec3(0.75, 0.86, 0.94), vec3(0.15, 0.34, 0.72), up);
    float s = max(dot(rd, sun), 0.0);
    sky += vec3(1.0, 0.9, 0.7) * pow(s, 220.0);          // sun disk
    sky += vec3(1.0, 0.7, 0.4) * pow(s, 6.0) * 0.25;      // warm glow
    return sky;
}

void main() {
    vec2 uv = (gl_FragCoord.xy - 0.5 * uResolution.xy) / uResolution.y;

    vec3 ro  = vec3(0.0, 2.6, uTime * 0.6);               // camera drifts forward over the water
    vec3 rd  = normalize(vec3(uv.x, uv.y - 0.28, -1.0));  // looking slightly down toward the horizon
    vec3 sun = normalize(vec3(0.35, 0.22, -1.0));

    // Sphere-trace the heightfield. tm/tx bracket the surface; refine by bisection each step.
    float tm = 0.0, tx = 40.0;
    float hm = map(ro + rd * tm);
    float hx = map(ro + rd * tx);
    float t  = tx;
    for (int i = 0; i < int(uSteps); i++) {
        float tmid = mix(tm, tx, hm / (hm - hx));
        float h = map(ro + rd * tmid);
        if (h < 0.0) { tx = tmid; hx = h; } else { tm = tmid; hm = h; }
        t = tmid;
        if (abs(h) < 0.01) break;
    }

    vec3 sky = getSkyColor(rd, sun);                      // computed once (used by sky rays + horizon haze)
    vec3 col;
    if (rd.y > -0.02 || hx > 0.0) {
        col = sky;                                        // ray escaped above the horizon
    } else {
        vec3 p = ro + rd * t;
        vec3 n = getNormal(p);
        vec3 rf = reflect(rd, n);                         // shared by the reflection and the specular glint
        float fres = pow(1.0 - max(dot(n, -rd), 0.0), 3.0) * 0.65 + 0.02;
        vec3 water = mix(vec3(0.0, 0.09, 0.16), vec3(0.10, 0.34, 0.42), max(n.y, 0.0));
        col = mix(water, getSkyColor(rf, sun), fres);     // sea colour <- sky reflection by Fresnel
        col += vec3(1.0, 0.95, 0.8) * pow(max(dot(rf, sun), 0.0), 120.0);     // sun glitter
        col = mix(sky, col, clamp(-rd.y * 12.0, 0.0, 1.0));                    // fade into horizon haze
    }

    col = pow(col, vec3(0.4545));                         // gamma
    gl_FragColor = vec4(col, 1.0);
}
