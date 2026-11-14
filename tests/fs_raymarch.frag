// fs_raymarch.frag — host/fb/fb_shader.py: a ray-marched sphere on a floor (tests/raymarch.frag's scene, the screen
// position from the vertex shader's varying instead of gl_FragCoord). Sphere tracing with a hardware LOOP of uSteps
// steps and a per-lane BREAK, central-difference normals (6 more SDF evaluations), Lambert + ambient, gamma 2.2.
// The sphere bobs with vUV.z (the time) so the picture moves; the step count is the measured variable.
precision mediump float;
uniform float uSteps;               // marching steps (runtime -> hardware loop)
varying vec4 vUV;                   // (x * aspect, y, time, 1)

float map(vec3 p, float t) {        // scene SDF = union(sphere, ground plane)
    float s     = length(p - vec3(0.0, 0.25 * sin(t), 0.0)) - 1.0;
    float plane = p.y + 1.0;
    return min(s, plane);
}

void main() {
    vec2 uv = vUV.xy * 0.5;
    float tm = vUV.z;
    vec3 ro = vec3(0.0, 0.6, 3.0);                   // camera
    vec3 rd = normalize(vec3(uv, -1.4));             // ray direction

    float t = 0.0;
    float hit = 0.0;
    for (int i = 0; i < int(uSteps); i++) {          // sphere tracing
        vec3  p = ro + rd * t;
        float d = map(p, tm);
        if (d < 0.002) { hit = 1.0; break; }         // surface hit
        t += d;
        if (t > 12.0) break;                         // escaped to background
    }

    vec3 col = vec3(0.55, 0.72, 0.95);               // sky gradient base
    col -= 0.3 * uv.y;
    if (hit > 0.5) {
        vec3 p = ro + rd * t;
        vec2 e = vec2(0.008, 0.0);
        vec3 n = normalize(vec3(map(p + e.xyy, tm) - map(p - e.xyy, tm),
                                map(p + e.yxy, tm) - map(p - e.yxy, tm),
                                map(p + e.yyx, tm) - map(p - e.yyx, tm)));
        vec3 lig = normalize(vec3(0.7, 0.8, 0.5));
        float dif = max(dot(n, lig), 0.0);
        float amb = 0.35;
        vec3 base = (p.y < -0.9) ? vec3(0.35, 0.55, 0.35) : vec3(0.95, 0.45, 0.30);
        col = base * (dif + amb);
    }
    col = pow(col, vec3(0.4545));                    // gamma 2.2
    gl_FragColor = vec4(col, 1.0);
}
