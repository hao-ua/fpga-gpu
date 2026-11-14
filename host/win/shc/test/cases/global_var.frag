precision mediump float;
varying vec4 v;
uniform float n;
vec3 shade;
float a = 1.0, b;
float list[3];
struct S { vec2 p; float q; };
S held;
void put(float x) { list[0] = x; list[1] = x + 1.0; held.q = x; }
float get() { return list[0] + list[1] + a; }
void main() {
    shade = v.xyz;
    b = n;
    put(v.x);
    if (v.y > 0.5) a = 3.0;
    held.p = shade.xy;
    gl_FragColor = vec4(shade * b + vec3(get(), list[1], held.q), held.p.x);
}
