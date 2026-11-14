precision mediump float;
varying float f[8]; varying float g[8]; varying float h[8]; varying float j[8];
void main() { gl_FragColor = vec4(f[0], f[7], g[0] + g[7], j[7] + h[1]); }
