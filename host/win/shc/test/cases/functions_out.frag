precision mediump float;
varying vec2 v;
uniform sampler2D t;
const float PI = 3.14159265;
const vec3 W = vec3(0.299, 0.587, 0.114);
float luma(vec3 c) { return dot(c, W); }
void split(in vec4 x, out vec3 rgb, out float a) { rgb = x.rgb; a = x.a; }
vec3 tone(vec3 c, float e) { vec3 r = c * e; return r / (r + vec3(1.0)); }
void main() {
    vec3 c; float a;
    split(texture2D(t, v), c, a);
    if (luma(c) < 0.25) discard;
    c = tone(c, 2.0);
    a = a > 0.5 ? a : a * a;
    gl_FragColor = vec4(c * sin(v.x * PI) * cos(v.y * PI), a);
}