precision mediump float; varying vec4 v0; uniform float n;
#if (1 || NOPE) && !(0 && NOPE)
void main() { gl_FragColor = vec4(1.0); }
#else
void main() { gl_FragColor = vec4(0.0); }
#endif
