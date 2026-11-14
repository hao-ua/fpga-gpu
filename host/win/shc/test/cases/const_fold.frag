precision highp float;
const float a = 1.0 / 3.0;
const float b = 0.1 + 0.2;
const float c = sqrt(2.0) * sqrt(2.0);
const float d = pow(2.0, 0.5);
const float e = exp(1.0) - 1.718281828;
const vec2 f = vec2(1.0, 2.0) * 3.0;
const vec3 g = vec3(f, a);
const mat2 m = mat2(1.0, 2.0, 3.0, 4.0);
const int n = 7 / 2;
const int o = (3 + 4) * 2 - 1;
const bool p = 1.0 < 2.0;
const float q = float(n) + mod(7.5, 2.0) + floor(-1.5) + ceil(1.2) + fract(2.75) + abs(-3.0) + sign(-2.0);
uniform vec4 u;
void main() {
    float r = p ? q : -q;
    gl_FragColor = vec4(a, b, c, d) * e + vec4(g, r) * u + vec4(m[1], f) + float(o);
}