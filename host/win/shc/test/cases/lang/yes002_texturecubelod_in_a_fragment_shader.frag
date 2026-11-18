precision mediump float; uniform vec4 u; uniform float f; uniform int ii; varying vec2 w;
uniform samplerCube s;
void main() { gl_FragColor = textureCubeLod(s, vec3(w, 1.0), f); }
