precision mediump float;
varying mat3 m;
varying mat2 q;
varying vec2 after;
void main() { gl_FragColor = vec4(m * vec3(1.0), 1.0) + vec4(q * after, 0.0, 0.0); }
