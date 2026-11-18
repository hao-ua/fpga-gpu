precision mediump float; varying vec4 v0; uniform float n;
#if (8 != 3 > 2) == 1 && (8 / ~2) == -2 && (7 % 3 << 1) == 2 && (1 | 2 ^ 3 & 1) == 3
void main() { gl_FragColor = vec4(1.0); }
#else
void main() { gl_FragColor = vec4(0.0); }
#endif
