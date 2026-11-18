precision mediump float; uniform vec4 u; uniform float f; uniform int ii; varying vec2 w;
void main() { gl_FragColor = vec4(dFdx(w), dFdy(w.x), fwidth(w.y)); }
