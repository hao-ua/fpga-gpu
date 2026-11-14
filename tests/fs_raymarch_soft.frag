// Desktop GPUs implement mediump as 32-bit float, so a raymarcher written
// against it looks correct there. Mobile GPUs implement it as a genuine 16-bit
// half, where distance accumulation and normal estimation lose enough precision
// to break up the surface into black speckle. Ask for highp where it exists.
#ifdef GL_FRAGMENT_PRECISION_HIGH
precision highp float;
#else
precision mediump float;
#endif
uniform vec2 u_Resolution;
uniform float u_Time;

#define MAX_STEPS 100
#define MAX_DIST 100.0
#define SURF_DIST 0.001

float sdSphere(vec3 p, float r)
{
    return length(p) - r;
}

float sdPlane(vec3 p, vec3 n, float h)
{
    return dot(p, n) + h;
}

float map(vec3 p)
{
    float sphere = sdSphere(p - vec3(0.0, 1.0, 0.0), 1.0);
    float plane  = sdPlane(p, vec3(0.0, 1.0, 0.0), 0.0);
    return min(sphere, plane);
}

vec3 getNormal(vec3 p)
{
    vec2 e = vec2(0.001, 0.0);
    float d = map(p);
    vec3 n = d - vec3(
        map(p - e.xyy),
        map(p - e.yxy),
        map(p - e.yyx)
    );
    return normalize(n);
}

float raymarch(vec3 ro, vec3 rd)
{
    float dO = 0.0;
    for (int i = 0; i < MAX_STEPS; i++)
    {
        vec3 p = ro + rd * dO;
        float dS = map(p);
        dO += dS;
        if (dO > MAX_DIST || abs(dS) < SURF_DIST) break;
    }
    return dO;
}

float softShadow(vec3 ro, vec3 rd, float mint, float maxt, float k)
{
    float res = 1.0;
    float t = mint;
    for (int i = 0; i < 48; i++)
    {
        float h = map(ro + rd * t);
        if (h < 0.001) return 0.0;
        res = min(res, k * h / t);
        t += clamp(h, 0.01, 0.2);
        if (t > maxt) break;
    }
    return res;
}

vec3 getMaterial(vec3 p)
{
    float sphereDist = sdSphere(p - vec3(0.0, 1.0, 0.0), 1.0);
    if (sphereDist < 0.01)
        return vec3(0.2, 0.6, 1.0);

    float checker = mod(floor(p.x) + floor(p.z), 2.0);
    return mix(vec3(0.15), vec3(0.85), checker);
}

mat3 lookAt(vec3 ro, vec3 ta)
{
    vec3 f = normalize(ta - ro);
    vec3 r = normalize(cross(vec3(0.0, 1.0, 0.0), f));
    vec3 u = cross(f, r);
    return mat3(r, u, f);
}

void main()
{
    vec2 uv = (gl_FragCoord.xy - 0.5 * u_Resolution.xy) / u_Resolution.y;

    float time = u_Time;

    // Camera
    vec3 ro = vec3(4.0 * sin(time * 0.5), 2.5, 4.0 * cos(time * 0.5));
    vec3 ta = vec3(0.0, 1.0, 0.0);
    mat3 cam = lookAt(ro, ta);
    vec3 rd = normalize(cam * vec3(uv, 1.5));

    // Background
    vec3 col = vec3(0.7, 0.9, 1.0) - rd.y * 0.2;

    float d = raymarch(ro, rd);
    if (d < MAX_DIST)
    {
        vec3 p = ro + rd * d;
        vec3 n = getNormal(p);
        vec3 mate = getMaterial(p);

        vec3 lightPos = vec3(3.0, 5.0, 2.0);
        vec3 l = normalize(lightPos - p);
        vec3 v = normalize(ro - p);
        vec3 h = normalize(l + v);

        float diff = max(dot(n, l), 0.0);
        float spec = pow(max(dot(n, h), 0.0), 64.0);
        float sh = softShadow(p + n * 0.01, l, 0.02, 10.0, 16.0);

        float ambient = 0.15;
        float fresnel = pow(1.0 - max(dot(n, v), 0.0), 3.0) * 0.25;

        col = mate * (ambient + diff * sh) + spec * sh + fresnel;

        // Simple fog
        col = mix(col, vec3(0.7, 0.9, 1.0), 1.0 - exp(-0.02 * d * d));
    }

    // Gamma correction
    col = pow(col, vec3(0.4545));

    gl_FragColor = vec4(col, 1.0);
}
