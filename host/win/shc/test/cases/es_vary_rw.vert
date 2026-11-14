attribute vec2 uv;
attribute vec4 p;
varying vec2 t;
varying vec4 w;
varying float f;
void main() { t = uv; t.y = 1.0 - t.y; w = p; f = w.x + t.x; w.xy *= 0.5; gl_Position = p; }
