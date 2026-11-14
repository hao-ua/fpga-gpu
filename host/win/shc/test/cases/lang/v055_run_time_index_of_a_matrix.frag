precision mediump float; varying vec4 v0; uniform float n;
void main() { int k = int(v0.x * 4.0); mat4 m = mat4(1.0); m[3][0] = 5.0; gl_FragColor = m[k]; }
