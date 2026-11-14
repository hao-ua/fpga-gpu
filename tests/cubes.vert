// cubes.vert — host/fb/fb_cube.py --cubes N (one draw for every cube): the rotation is computed per vertex from the
// global angle (uAnim.x) and the cube's phase (aInst.w) with the hardware SIN / COS, R = Ry(a) * Rx(0.7 a); the cube is
// scaled by uAnim.y and placed at aInst.xyz; the view-projection comes as rows (uRow0..3). Lighting as cube.vert.
attribute vec3 aPos;
attribute vec3 aNormal;
attribute vec3 aColor;
attribute vec4 aInst;
uniform vec4 uRow0;
uniform vec4 uRow1;
uniform vec4 uRow2;
uniform vec4 uRow3;
uniform vec4 uLight;
uniform vec4 uAnim;
varying vec4 vColor;
void main() {
    float a = uAnim.x + aInst.w;
    float b = uAnim.z * a;
    float cy = cos(a);
    float sy = sin(a);
    float cx = cos(b);
    float sx = sin(b);
    vec3 r0 = vec3(cy, sy * sx, sy * cx);
    vec3 r1 = vec3(0.0, cx, -sx);
    vec3 r2 = vec3(-sy, cy * sx, cy * cx);
    vec3 p = vec3(dot(r0, aPos), dot(r1, aPos), dot(r2, aPos)) * uAnim.y + aInst.xyz;
    vec4 pw = vec4(p, 1.0);
    gl_Position = vec4(dot(uRow0, pw), dot(uRow1, pw), dot(uRow2, pw), dot(uRow3, pw));
    vec3 n = vec3(dot(r0, aNormal), dot(r1, aNormal), dot(r2, aNormal));
    float d = max(dot(n, uLight.xyz), 0.0);
    vColor = vec4(aColor * (uLight.w + (1.0 - uLight.w) * d), 1.0);
}
