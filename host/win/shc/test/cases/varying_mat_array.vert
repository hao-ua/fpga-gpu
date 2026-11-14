// an array of matrix varyings: one varying vector per column of each element, read in the fragment stage by constant index
attribute vec4 a;
varying mat2 m[2];
varying mat3 n[2];
varying vec4 w;
void main() {
    m[0] = mat2(a.x, a.y, a.z, a.w);
    m[1] = mat2(1.0);
    n[1] = mat3(a.xyz, a.yzx, a.zxy);
    n[0] = mat3(0.5);
    w = a;
    gl_Position = a;
}
