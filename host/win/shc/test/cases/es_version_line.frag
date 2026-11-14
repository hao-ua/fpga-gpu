#if __VERSION__ != 100
#error wrong version
#endif
#ifdef GL_FRAGMENT_PRECISION_HIGH
precision highp float;
#else
precision mediump float;
#endif
#if __LINE__ != 9
#error wrong line
#endif
void main() { gl_FragColor = vec4(float(__LINE__), float(__FILE__), float(__VERSION__), 1.0); }
