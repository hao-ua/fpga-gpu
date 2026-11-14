attribute mat4 m;
attribute mat2 r;
attribute vec4 p;
varying vec2 v;
void main() { v = r * p.xy; gl_Position = m * p; }
