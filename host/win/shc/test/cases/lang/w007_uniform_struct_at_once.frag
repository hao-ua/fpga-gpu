precision mediump float; varying vec4 v0; uniform float n;
uniform struct U { float k; } u;
void main() { gl_FragColor = vec4(u.k); }
