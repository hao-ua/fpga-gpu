// local arrays of matrices and of structs: every element its own registers, written and read by constant index
struct L { vec2 v; mat2 m; };
attribute vec4 a;
void main() {
    mat2 r[2];
    mat3 t[2];
    L l[2];
    r[0] = mat2(a.x, a.y, a.z, a.w);
    r[1] = r[0] * r[0];
    r[1][0].x = 2.0;
    t[1] = mat3(1.0);
    l[1].v = a.xy;
    l[1].m = r[1];
    l[0] = l[1];
    gl_Position = vec4(r[1] * l[0].v, t[1][2].z, l[0].m[1][1]);
}
