precision mediump float;
varying vec2 v;
uniform sampler2D s[2];
void main() { gl_FragColor = texture2D(s[2], v); }
