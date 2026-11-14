precision mediump float; varying vec4 v0; uniform float n;
void main() { vec4 w = vec4(1.0, 2.0, 3.0, 4.0); w.wz.y = 9.0; gl_FragColor = vec4(w.xy.yx.xy, w.zw); }
