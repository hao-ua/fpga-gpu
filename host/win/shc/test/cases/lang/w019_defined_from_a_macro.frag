precision mediump float; varying vec4 v0; uniform float n;
#define D defined(NOPE)
#if !D
void main() { gl_FragColor = vec4(1.0); }
#else
void main() { gl_FragColor = vec4(0.0); }
#endif
