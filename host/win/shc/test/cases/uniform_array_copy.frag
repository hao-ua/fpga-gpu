precision mediump float;
struct S { vec4 a[3]; float f; };
uniform S u;
void main() {
    S s;
    s = u;
    s.a[0] = s.a[0] * 0.5;
    gl_FragColor = s.a[0] + s.a[1] * s.f + s.a[2];
}
