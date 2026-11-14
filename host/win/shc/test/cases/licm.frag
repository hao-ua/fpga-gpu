// licm.frag — loop-invariant code motion out of hardware loops (glslc.licm / shc_opt.c licm), the copy back into a loop's
// variable (_copy_back / copy_back), a vec3 written as three components inside a loop (narrow_masks' _exposed), abs() as a source
// modifier (block_forward), dot with a basis vector and + 0.0 through a parameter (bi_dot / binop). With the shapes each must leave.
precision mediump float;
varying vec4 v0;
uniform float n;

float plane(vec3 p, vec3 nn, float h) { return dot(p, nn) + h; }

float hoist(float x)   { float s = 0.0; for (int i = 0; i < 30; i++) { if (x > 0.5) break; vec3 c = vec3(1.0, 2.0, 3.0) * n; s += c.y; } return s; }
float after(float x)   { float s = 0.0; float k = 1.0; for (int i = 0; i < 20; i++) { if (x > 0.5 && s > 1.5) break; k = n * 2.0; s += 1.0; } return s + k; }
float accum(float x)   { float s = 0.0; for (int i = 0; i < 20; i++) { s = s + n; if (x > 0.5 && s > 10.0) break; } return s; }
float before(float x)  { float s = 0.0; float t = 0.0; for (int i = 0; i < 20; i++) { s += t; t = n * 2.0; if (x > 0.5 && s > 10.0) break; } return s; }
float underif(float x) { float s = 0.0; for (int i = 0; i < 20; i++) { if (x > 0.5) { float c = n * 0.5; s += c; } else s += 1.0; } return s; }
float inner(float x)   { float s = 0.0; for (int i = 0; i < 20; i++) { for (int j = 0; j < 20; j++) { s += dot(vec2(1.0, 2.0), vec2(n)); if (x > 0.5 && j == 1) break; } if (x > 0.5) break; } return s; }
float rt(float x)      { float s = 0.0; for (float i = 0.0; i < n; i += 1.0) { vec2 c = vec2(3.0, 5.0) * n; s += c.x - c.y; if (x > 0.5) break; } return s; }
float march(vec3 ro, vec3 rd)
{
    float d = 0.0;
    for (int i = 0; i < 80; i++) {
        vec3 p = ro + rd * d;
        float ds = min(length(p - vec3(0.0, 1.0, 0.0)) - 1.0, plane(p, vec3(0.0, 1.0, 0.0), 0.0));
        d += ds;
        if (d > 50.0 || abs(ds) < 0.001) break;
    }
    return d;
}

void main()
{
    float x = v0.x;
    float a = hoist(x) + after(x) + accum(x) + before(x) + underif(x);
    float b = inner(x) + rt(x) + march(vec3(0.0, 1.5, -4.0), normalize(vec3(x - 0.5, -0.2, 1.0)));
    gl_FragColor = vec4(a, b, -abs(x - 0.5) + abs(-x), 1.0);
}
