precision mediump float;
uniform float u[40]; uniform float q[40]; uniform vec2 w[30]; uniform vec3 x[4]; uniform vec4 y; varying vec2 t;
void main() { gl_FragColor = vec4(u[0] + u[39] + q[1], w[29].y, x[3].z + y.w, t.x); }
