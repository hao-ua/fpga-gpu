precision mediump float; varying vec4 v0; uniform float n;
void main() { vec4 x = vec4(-3.9, -0.6, 1.7, 3.95); vec3 y = atan(x.wzy); gl_FragColor = atan(x) + vec4(0.0, 0.0, 0.0, y.x - atan(3.95)); }
