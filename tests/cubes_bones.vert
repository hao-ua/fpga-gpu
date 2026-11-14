// cubes_bones.vert — host/fb/fb_cube.py --bones: one draw for every cube, each cube's MODEL MATRIX picked per vertex
// from the uniform array uBone[NB] with a RUN-TIME index (the cube number in aInst.w): the ES2 vertex-shader uniform
// indexing (Appendix A.5; the 64-bit core's LDPX). The view-projection comes as rows (uRow0..3); lighting as cube.vert.
// NB is #defined by the host (the cube count).
attribute vec3 aPos;
attribute vec3 aNormal;
attribute vec3 aColor;
attribute vec4 aInst;
uniform vec4 uRow0;
uniform vec4 uRow1;
uniform vec4 uRow2;
uniform vec4 uRow3;
uniform vec4 uLight;
uniform vec4 uAnim;                 // .y = 1 / the cubes' scale: the model matrices scale, the normal is unscaled with it
uniform mat4 uBone[NB];
varying vec4 vColor;
void main() {
    int k = int(aInst.w);
    mat4 M = uBone[k];
    vec4 pw = M * vec4(aPos, 1.0);
    gl_Position = vec4(dot(uRow0, pw), dot(uRow1, pw), dot(uRow2, pw), dot(uRow3, pw));
    vec3 n = (M * vec4(aNormal, 0.0)).xyz * uAnim.y;
    float d = max(dot(n, uLight.xyz), 0.0);
    vColor = vec4(aColor * (uLight.w + (1.0 - uLight.w) * d), 1.0);
}
