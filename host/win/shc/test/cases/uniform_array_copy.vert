// a struct holding uniform arrays copied into a local: every element is loaded, and the copy can be written
struct E { vec2 v; float k; };
struct S { vec4 a[2]; mat2 m[2]; E e[2]; float f; };
uniform S u;
attribute vec4 p;
void main() {
    S s;
    s = u;
    s.a[1] = s.a[1] + p;
    s.e[1].k = s.e[1].k * 2.0;
    gl_Position = s.a[0] + s.a[1] + vec4(s.m[1][0], s.f, s.e[1].k + s.e[0].v.y);
}
