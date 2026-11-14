// fs_quad.vert — host/fb/fb_shader.py: a full-screen quad. aPos = the corner in NDC (-1..1); the varying carries the
// aspect-corrected screen position (x in -aspect..aspect, y in -1..1) and the frame's time for the fragment shader.
attribute vec2 aPos;
uniform vec4 uParam;                 // (aspect, time, 0, 0)
varying vec4 vUV;
void main() {
    gl_Position = vec4(aPos, 0.0, 1.0);
    vUV = vec4(aPos.x * uParam.x, aPos.y, uParam.y, 1.0);
}
