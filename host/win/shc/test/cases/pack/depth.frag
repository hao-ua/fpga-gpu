precision mediump float;
uniform sampler2D z;
uniform sampler2D c;
varying vec2 t;
void main() { gl_FragColor = texture2D(z, t) + texture2D(c, t); }
