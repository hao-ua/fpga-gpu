// tex3d_demo.vert — host/fb/fb_tex3d_demo.py: the volume's slices, stored in the volume's own coordinates (aPos, -1 .. 1
// each), turned to the screen by the rows uR0, uR1; the texture coordinate is the position itself.
attribute vec3 aPos;
uniform vec4 uScale;                 // (x scale, y scale, 0, 0): view space to the screen
uniform vec4 uR0;
uniform vec4 uR1;
varying vec4 vUV;
void main() {
    gl_Position = vec4(dot(uR0.xyz, aPos) * uScale.x, dot(uR1.xyz, aPos) * uScale.y, 0.0, 1.0);
    vUV = vec4(aPos * 0.5 + vec3(0.5), 1.0);
}
