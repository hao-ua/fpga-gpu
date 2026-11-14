precision mediump float; varying vec4 v0; uniform float n;
void main() { gl_FragColor = vec4(dot(2.0, 3.0), length(-2.0), distance(1.0, 4.0), normalize(-3.0)); }
