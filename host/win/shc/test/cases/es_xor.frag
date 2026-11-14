precision mediump float;
varying vec4 v;
const bool k = true ^^ false;
void main() { bool a = v.x > 0.5, b = v.y > 0.5; bool c = a ^^ b ^^ k; if (a && b ^^ c || a) discard; gl_FragColor = c ? v : v * 0.5; }
