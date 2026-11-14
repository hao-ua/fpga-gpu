precision mediump float; varying vec4 v0; uniform float n;
void main() { for (int i = 0; i < gl_MaxDrawBuffers; i++) gl_FragData[i] = vec4(1.0, 2.0, 3.0, 4.0); }
