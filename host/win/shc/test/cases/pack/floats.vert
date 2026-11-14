attribute vec4 a0;
varying float f[8]; varying float g[8]; varying float h[8]; varying float j[8];
void main() { gl_Position = a0; for (int i = 0; i < 8; i++) { f[i] = float(i); g[i] = float(i + 8); h[i] = float(i + 16); j[i] = float(i + 24); } }
