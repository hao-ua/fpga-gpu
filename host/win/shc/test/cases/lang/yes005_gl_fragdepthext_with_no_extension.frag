precision mediump float; uniform vec4 u; uniform float f; uniform int ii; varying vec2 w;
void main() { gl_FragDepthEXT = f; gl_FragColor = u; }
