precision mediump float; uniform vec4 u; uniform float f; uniform int ii; varying vec2 w;
uniform sampler2D s;
void main() { gl_FragColor = texture2DLod(s, w, f); }
