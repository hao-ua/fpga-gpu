// "Kitchen sink" GLSL ES 1.00 fragment shader — exercises nearly every compiler feature in one program,
// so compiling it and running the emitted ISA on the real FS core RTL confirms the hardware executes it
// correctly (vs the shader_ref golden). Texture ops are excluded (they need the TMU; verified separately).
precision mediump float;

struct Light { vec3 dir; vec3 color; float intensity; };

uniform Light lights[3];      // array of structs
uniform int   numLights;      // int uniform (FTOI-on-load)
uniform vec4  uParams;        // x=threshold y=eta/facing z=specExp w=mixAmt/atten
uniform vec4  uBase;
uniform mat3  uRot;           // matrix uniform

varying vec3 vNormal;         // per-thread -> SIMT divergence
varying vec4 vSelector;

const int   MAX_LIGHTS = 3;               // global const scalar (loop bound)
const vec3  UP = vec3(0.0, 1.0, 0.0);     // global const vector
const float PI = 3.14159265;              // global const float chain
const float HALF_PI = PI * 0.5;

float attenuate(float d, in float k) { return 1.0 / (1.0 + k * d * d); }   // 'in' param
void  accumulate(inout vec3 acc, in vec3 c, in float w) { acc += c * w; }  // 'inout' param

void main() {
    vec3 N = normalize(uRot * vNormal);                 // mat3 * vec3
    vec3 acc = vec3(0.0);

    // const-bounded FOR with runtime break + array-of-structs indexing by loop var
    for (int i = 0; i < MAX_LIGHTS; i++) {
        if (i >= numLights) break;
        vec3  L    = normalize(-lights[i].dir);
        float ndl  = max(dot(N, L), 0.0);
        float spec = pow(ndl, uParams.z);
        accumulate(acc, lights[i].color * lights[i].intensity,
                   (ndl + spec) * attenuate(float(i) + 1.0, uParams.w));
    }

    // int counter + do-while + divergent-safe continue + sin/cos
    int cnt = 0; float ripple = 0.0; int j = 0;
    do {
        j++;
        if (j == 2) continue;
        ripple += sin(float(j) * HALF_PI) + cos(float(j));
        cnt++;
    } while (j < 4);

    // while loop (const bound) with int counter
    float poly = 0.0; int k = 0;
    while (k < 3) { poly += float(k); k = k + 1; }

    // built-in inputs + ternary + scalar-swizzle broadcast (divergent per thread)
    float fog    = clamp(gl_FragCoord.z, 0.0, 1.0);
    float sel    = vSelector.x;
    vec3  tint   = (sel > uParams.x) ? uBase.rgb : UP;
    float facing = gl_FrontFacing ? 1.0 : uParams.y;

    // geometric + common builtins
    vec3  rfr = refract(normalize(vNormal), N, uParams.y);
    vec3  rfl = reflect(N, UP);
    vec3  crs = cross(N, UP);
    float mag = length(acc) + abs(sel) + floor(poly) + fract(PI);

    // compose: compound assign, chained assign, comma, mix/step/smoothstep/sign
    vec3 color = acc * facing + tint * fog;
    color *= (1.0 + ripple);
    float a, b; a = b = poly + float(cnt);          // chained assignment
    float extra = (mag, a + b);                     // comma -> a+b
    float edge  = step(uParams.x, sel) + smoothstep(0.0, 1.0, fog) + sign(sel);
    color += mix(rfr, rfl, uParams.w) + crs + vec3(extra + edge);

    if (sel < -0.5) discard;                        // divergent discard

    gl_FragColor = vec4(color, 1.0);
}
