precision mediump float;
varying vec2 v;
uniform sampler2D s[3];
uniform sampler2D after;
const int k = 2;
void main() { vec4 c = vec4(0.0); for (int i = 0; i < 3; i++) c += texture2D(s[i], v); gl_FragColor = c + texture2D(s[k], v) + texture2D(after, v); }
