uniform sampler2D s; attribute vec2 a;
void main() { gl_Position = texture2D(s, a, 1.0); }
