// vs_skin — run-time uniform indexing in the vertex shader (ES2 Appendix A.5: uniform arrays may be indexed by any
// integer expression there): a bone matrix picked per vertex, a colour row and a struct element picked from a
// derived index. glslc64_check pins the index per vertex for the 32-bit reference (the directive below).
// glslc64_check: pin a_bone int(a*3.5+4.0)
attribute vec4 a_pos;
attribute float a_bone;
attribute vec2 a_uv;
uniform mat4 uMVP;
uniform mat4 uBone[8];
uniform vec4 uCol[6];
struct Light { vec3 dir; vec4 k; };               // (a scalar member would read .x only on the 64-bit core, the
uniform Light uL[4];                              //  packer's convention; the 32-bit reference broadcasts the slot)
varying vec4 v_col;
varying vec2 v_uv;
#ifdef PIN
#define BONE PIN
#else
#define BONE int(a_bone * 3.5 + 4.0)
#endif
void main() {
    int b = BONE;                                   // 0..7
    vec4 p = uBone[b] * a_pos;
    gl_Position = uMVP * p;
    int h = b / 2;                                  // 0..3
    v_col = uCol[h + 2] * uL[h].k + vec4(uL[h].dir, 0.0);
    v_uv = a_uv;
}
