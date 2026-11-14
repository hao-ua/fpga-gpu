precision mediump float; uniform vec4 u; uniform float f; uniform int ii; varying vec2 w;
uniform sampler2D s;
void main() { vec4 c = texture2DLod(s, w, 0.0); }
