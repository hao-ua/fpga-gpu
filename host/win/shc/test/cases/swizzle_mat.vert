attribute vec4 p;
attribute vec2 uv;
uniform mat4 m;
uniform mat3 n;
uniform mat2 r;
uniform vec4 arr[4];
uniform mat4 bones[3];
varying vec4 a;
varying vec2 b;
varying float c;
void main() {
    vec4 q = p.wzyx * p.xxyy + p.xyzw;
    q.xz = q.zx;
    q.w *= 2.0;
    mat4 t = m;
    t[2][1] = 0.5;
    t[3] = arr[1];
    vec3 w = n * q.xyz;
    w = w * n;
    b = r * uv + uv * r;
    a = bones[1] * vec4(w, 1.0) + (bones[2] * t) * q;
    c = length(a.xy) + distance(a, q) + dot(uv, b);
    gl_PointSize = c;
    gl_Position = t * q;
}