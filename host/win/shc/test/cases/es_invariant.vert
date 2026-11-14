invariant gl_Position;
invariant varying vec4 v;
varying vec4 w;
invariant w;
attribute vec4 p;
void main() { v = p; w = p * 2.0; gl_Position = p; }
