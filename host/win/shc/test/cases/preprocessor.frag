#version 100
#ifdef GL_ES
precision mediump float;
#endif
#define A 2
#define B (A * 3 + 1)
#define MUL(x, y) ((x) * (y))
#if defined(A) && B == 7 && !defined(C)
#define OK 1.0
#else
#define OK 0.0
#endif
#if 0
garbage that must not reach the parser ( { [
#elif A > 1
#define E 0.25
#else
#define E 0.75
#endif
#undef A
varying vec4 v;
/* block
   comment */
void main() { // line comment
    gl_FragColor = v * MUL(OK, E);
}