attribute vec4 a; uniform sampler2D s;
void main() { gl_Position = texture2DLodEXT(s, a.xy, 0.0); }
