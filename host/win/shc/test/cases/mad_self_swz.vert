// v = c - k * v.qpst with v read before any write (0, a varying kept in a local) and written again under an if: the MUL reads
// v's register swizzled and the SUB writes it; fused, one MAD would write x before its micro-op for z reads it
// (isa64.check_overlap), so the pair stays apart (dEQP-GLES2 shaders.random.all_features.vertex.82)
attribute float r;
attribute vec4 f;
uniform mediump vec4 h;
uniform bool k;
varying mediump vec4 v;
void main() {
    v = vec4(r, 10.25, -1.25, 0.0).barg - vec4(-16.0, -3.5, -1.0, -4.5) * v.qpst;
    if (k) v = h - f.ptsq;
    gl_Position = f;
}
