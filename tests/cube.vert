// cube.vert — host/fb/fb_cube.py: a rotating cube lit by one directional light (Lambert + ambient), computed per vertex.
// The matrices come as rows (vec4 uniforms), so the uniform layout is explicit: MVP rows 0..3, the model rotation's
// rows 4..6 (the normal matrix: a pure rotation), uLight = (light direction in world space, normalised; ambient).
attribute vec3 aPos;
attribute vec3 aNormal;
attribute vec3 aColor;
uniform vec4 uRow0;
uniform vec4 uRow1;
uniform vec4 uRow2;
uniform vec4 uRow3;
uniform vec4 uNRow0;
uniform vec4 uNRow1;
uniform vec4 uNRow2;
uniform vec4 uLight;
varying vec4 vColor;
void main() {
    vec4 p = vec4(aPos, 1.0);
    gl_Position = vec4(dot(uRow0, p), dot(uRow1, p), dot(uRow2, p), dot(uRow3, p));
    vec3 n = vec3(dot(uNRow0.xyz, aNormal), dot(uNRow1.xyz, aNormal), dot(uNRow2.xyz, aNormal));
    float d = max(dot(n, uLight.xyz), 0.0);
    vColor = vec4(aColor * (uLight.w + (1.0 - uLight.w) * d), 1.0);
}
