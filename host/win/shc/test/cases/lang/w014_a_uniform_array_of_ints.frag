precision mediump float; varying vec4 v0; uniform float n;
uniform int a[2];
void main() { gl_FragColor = vec4(float(a[1] + 1)); }
